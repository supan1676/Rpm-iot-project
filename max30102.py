"""
MAX30102 — Pulse Oximeter & Heart Rate Sensor Driver for MicroPython (I2C)
Measures BPM and SpO₂ via optical PPG (Red + IR LEDs).

I2C Address: 0x57
Datasheet: https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX30102.html
"""

from micropython import const
import time

# Register addresses
_REG_INTR_STATUS_1  = const(0x00)
_REG_INTR_STATUS_2  = const(0x01)
_REG_INTR_ENABLE_1  = const(0x02)
_REG_INTR_ENABLE_2  = const(0x03)
_REG_FIFO_WR_PTR    = const(0x04)
_REG_OVF_COUNTER    = const(0x05)
_REG_FIFO_RD_PTR    = const(0x06)
_REG_FIFO_DATA      = const(0x07)
_REG_FIFO_CONFIG    = const(0x08)
_REG_MODE_CONFIG    = const(0x09)
_REG_SPO2_CONFIG    = const(0x0A)
_REG_LED1_PA        = const(0x0C)  # Red LED pulse amplitude
_REG_LED2_PA        = const(0x0D)  # IR LED pulse amplitude
_REG_PILOT_PA       = const(0x10)
_REG_MULTI_LED_1    = const(0x11)
_REG_MULTI_LED_2    = const(0x12)
_REG_TEMP_INT       = const(0x1F)
_REG_TEMP_FRAC      = const(0x20)
_REG_TEMP_CONFIG    = const(0x21)
_REG_REV_ID         = const(0xFE)
_REG_PART_ID        = const(0xFF)

# Expected part ID
_PART_ID_MAX30102 = const(0x15)


class MAX30102:
    """MAX30102 pulse oximeter driver for BPM and SpO₂ measurement."""

    def __init__(self, i2c, addr=0x57):
        self.i2c = i2c
        self.addr = addr

        # Verify part ID
        part_id = self._read_reg(_REG_PART_ID)
        if part_id != _PART_ID_MAX30102:
            raise RuntimeError(f"MAX30102 not found (PART_ID=0x{part_id:02X})")

        self._setup()

        # Buffers for BPM calculation
        self._ir_buffer = []
        self._red_buffer = []
        self._last_beat_time = 0
        self._bpm = 0
        self._spo2 = 0
        self._finger_detected = False

    def _read_reg(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def _write_reg(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _setup(self):
        """Configure sensor for SpO₂ mode with optimized settings."""
        # Reset the device
        self._write_reg(_REG_MODE_CONFIG, 0x40)
        time.sleep_ms(100)

        # Clear FIFO pointers
        self._write_reg(_REG_FIFO_WR_PTR, 0x00)
        self._write_reg(_REG_OVF_COUNTER, 0x00)
        self._write_reg(_REG_FIFO_RD_PTR, 0x00)

        # FIFO config: sample avg = 4, FIFO rollover enabled
        self._write_reg(_REG_FIFO_CONFIG, 0x4F)

        # SpO₂ mode (Red + IR)
        self._write_reg(_REG_MODE_CONFIG, 0x03)

        # SpO₂ config: ADC range 4096nA, 100 samples/s, 411μs pulse width
        self._write_reg(_REG_SPO2_CONFIG, 0x27)

        # LED pulse amplitudes (~10.6mA — optimal for bare finger optical reflection)
        self._write_reg(_REG_LED1_PA, 0x34)   # Red
        self._write_reg(_REG_LED2_PA, 0x34)   # IR

        # Enable data ready interrupt
        self._write_reg(_REG_INTR_ENABLE_1, 0xC0)
        self._write_reg(_REG_INTR_ENABLE_2, 0x00)

        # Clear interrupt status
        self._read_reg(_REG_INTR_STATUS_1)

    def _read_fifo(self):
        """Read one sample (Red + IR) from FIFO. Returns (red, ir) or (0, 0)."""
        try:
            data = self.i2c.readfrom_mem(self.addr, _REG_FIFO_DATA, 6)
            red = ((data[0] << 16) | (data[1] << 8) | data[2]) & 0x03FFFF
            ir  = ((data[3] << 16) | (data[4] << 8) | data[5]) & 0x03FFFF
            return red, ir
        except OSError:
            return 0, 0

    def drain_fifo(self):
        """
        Drain all pending samples from the MAX30102 hardware FIFO.
        Prevents FIFO overflow and maintains 25 SPS continuity.
        Returns number of new samples processed.
        """
        try:
            wr_ptr = self._read_reg(_REG_FIFO_WR_PTR)
            rd_ptr = self._read_reg(_REG_FIFO_RD_PTR)
        except OSError:
            return 0

        num_samples = (wr_ptr - rd_ptr) & 0x1F
        if num_samples == 0:
            return 0

        try:
            raw_data = self.i2c.readfrom_mem(self.addr, _REG_FIFO_DATA, num_samples * 6)
        except OSError:
            return 0

        samples_processed = 0
        for i in range(0, len(raw_data), 6):
            if i + 6 > len(raw_data):
                break
            red = ((raw_data[i] << 16) | (raw_data[i+1] << 8) | raw_data[i+2]) & 0x03FFFF
            ir  = ((raw_data[i+3] << 16) | (raw_data[i+4] << 8) | raw_data[i+5]) & 0x03FFFF

            # Finger detection threshold: IR channel > 35,000 counts
            finger_now = (ir > 35000)

            if not finger_now:
                self._finger_detected = False
                self._bpm = 0
                self._spo2 = 0
                self._ir_buffer.clear()
                self._red_buffer.clear()
            else:
                self._finger_detected = True
                self._ir_buffer.append(ir)
                self._red_buffer.append(red)
                if len(self._ir_buffer) > 150:
                    self._ir_buffer.pop(0)
                    self._red_buffer.pop(0)
                samples_processed += 1

        if self._finger_detected and len(self._ir_buffer) >= 50:
            self._calculate_bpm()
            self._calculate_spo2()

        return samples_processed

    def read_sensor(self):
        """
        Read sensor and update BPM/SpO₂ estimates.
        Drains FIFO to capture all available samples.
        """
        n = self.drain_fifo()
        if n > 0 and self._finger_detected:
            return True
        elif not self._finger_detected:
            # Check single sample if FIFO reported 0
            red, ir = self._read_fifo()
            self._finger_detected = (ir > 35000)
            if not self._finger_detected:
                self._bpm = 0
                self._spo2 = 0
        return self._finger_detected

    def _calculate_bpm(self):
        """Estimate BPM from IR signal using peak detection with refractory blanking."""
        buf = self._ir_buffer
        n = len(buf)
        if n < 50:
            return

        # DC removal: compute mean
        mean = sum(buf) // n
        max_val = max(buf)
        min_val = min(buf)

        # Pulse amplitude sanity check (reject ambient optical noise)
        if (max_val - min_val) < 400:
            return

        threshold = mean + int((max_val - mean) * 0.35)

        # Peak detection with refractory blanking (min 7 samples = ~214 BPM at 25 SPS)
        # Prevents dicrotic notch double-counting
        peaks = []
        last_peak = -999
        for i in range(2, n - 2):
            if (i - last_peak) < 7:
                continue
            val = buf[i]
            if val > threshold and val > buf[i - 1] and val > buf[i + 1] and val >= buf[i - 2] and val >= buf[i + 2]:
                peaks.append(i)
                last_peak = i

        # Calculate BPM from peak intervals
        if len(peaks) >= 2:
            intervals = [peaks[i+1] - peaks[i] for i in range(len(peaks) - 1)]
            # Valid human heart rate interval at 25 SPS (7 to 38 samples -> 40 to 214 BPM)
            valid = [iv for iv in intervals if 7 <= iv <= 38]
            if valid:
                avg_interval = sum(valid) / len(valid)
                calc_bpm = int(60 * 25 / avg_interval)
                calc_bpm = max(40, min(200, calc_bpm))
                # Smooth filter to prevent erratic jumping
                if self._bpm == 0:
                    self._bpm = calc_bpm
                else:
                    self._bpm = int(0.7 * self._bpm + 0.3 * calc_bpm)

    def _calculate_spo2(self):
        """Estimate SpO₂ from Red/IR ratio (Beer-Lambert approximation)."""
        red_buf = self._red_buffer[-50:]
        ir_buf = self._ir_buffer[-50:]

        red_dc = sum(red_buf) / len(red_buf)
        ir_dc = sum(ir_buf) / len(ir_buf)

        if red_dc < 1000 or ir_dc < 1000:
            return

        red_ac = (sum((r - red_dc) ** 2 for r in red_buf) / len(red_buf)) ** 0.5
        ir_ac = (sum((i - ir_dc) ** 2 for i in ir_buf) / len(ir_buf)) ** 0.5

        if ir_ac < 15 or red_ac < 15:
            return

        ratio = (red_ac / red_dc) / (ir_ac / ir_dc)

        # Empirical calibration curve: SpO₂ = 110 - 25 * R
        calc_spo2 = int(110 - 25 * ratio)
        calc_spo2 = max(75, min(100, calc_spo2))

        if self._spo2 == 0:
            self._spo2 = calc_spo2
        else:
            self._spo2 = int(0.8 * self._spo2 + 0.2 * calc_spo2)

    @property
    def bpm(self):
        """Current heart rate in BPM (0 if no finger detected)."""
        return self._bpm

    @property
    def spo2(self):
        """Current SpO₂ percentage (0 if no finger detected)."""
        return self._spo2

    @property
    def finger_on(self):
        """True if a finger is detected on the sensor."""
        return self._finger_detected

    def read_temperature(self):
        """Read die temperature in °C (useful for calibration)."""
        self._write_reg(_REG_TEMP_CONFIG, 0x01)
        time.sleep_ms(50)
        temp_int = self._read_reg(_REG_TEMP_INT)
        temp_frac = self._read_reg(_REG_TEMP_FRAC)
        return temp_int + (temp_frac * 0.0625)

    def shutdown(self):
        """Put sensor in low-power shutdown mode."""
        self._write_reg(_REG_MODE_CONFIG, 0x80)
