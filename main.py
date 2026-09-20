"""
IoMT Remote Patient Monitoring (RPM) Node — All-In-One Standalone MicroPython
ESP32-S3 DevKitC-1

Everything included in this single file:
  - SSD1306 128x64 OLED driver
  - MPU6050 6-Axis IMU driver (fall detection at ±8g)
  - MLX90614 Contactless IR Thermometer driver
  - MAX30102 Pulse Oximeter driver (PPG Heart Rate & SpO2)
  - Clinical alert engine, 25Hz fast tick loop, haptics & JSON telemetry

Wiring (ESP32-S3):
  GPIO 8  -> I2C SDA (OLED, MPU6050, MLX90614, MAX30102)
  GPIO 9  -> I2C SCL (OLED, MPU6050, MLX90614, MAX30102)
  GPIO 5  -> SOS Pushbutton (Input with internal pull-up, Active LOW)
  GPIO 18 -> Vibration Motor (via MOSFET gate)
  3.3V    -> Sensor VCC / VIN
  GND     -> Sensor GND (and MPU6050 AD0)

Hardware Tip:
  External 4.7kΩ pull-up resistors on SDA and SCL to 3.3V are strongly
  recommended for 4-device breadboard buses to ensure crisp I2C edges.

Disclaimer:
  This firmware is an IoMT engineering prototype/demonstration project.
  The empirical SpO2 algorithm (110 - 25*R) is a standard research approximation
  and is not intended for certified clinical or medical diagnosis.
"""

import framebuf
import json
import struct
import time
from machine import Pin, SoftI2C
from micropython import const

# ═════════════════════════════════════════════════════════════════════
# 1. SSD1306 OLED DRIVER (128x64)
# ═════════════════════════════════════════════════════════════════════

_SET_CONTRAST        = const(0x81)
_SET_ENTIRE_ON       = const(0xA4)
_SET_NORM_INV        = const(0xA6)
_SET_DISP            = const(0xAE)
_SET_MEM_ADDR        = const(0x20)
_SET_COL_ADDR        = const(0x21)
_SET_PAGE_ADDR       = const(0x22)
_SET_DISP_START_LINE = const(0x40)
_SET_SEG_REMAP       = const(0xA0)
_SET_MUX_RATIO       = const(0xA8)
_SET_COM_OUT_DIR     = const(0xC0)
_SET_DISP_OFFSET     = const(0xD3)
_SET_COM_PIN_CFG     = const(0xDA)
_SET_DISP_CLK_DIV    = const(0xD5)
_SET_PRECHARGE       = const(0xD9)
_SET_VCOM_DESEL      = const(0xDB)
_SET_CHARGE_PUMP     = const(0x8D)
_SET_IREF_SELECT     = const(0xAD)


class SSD1306(framebuf.FrameBuffer):
    def __init__(self, width, height, external_vcc=False):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self.init_display()

    def init_display(self):
        for cmd in (
            _SET_DISP,
            _SET_MEM_ADDR, 0x00,
            _SET_DISP_START_LINE | 0x00,
            _SET_SEG_REMAP | 0x01,
            _SET_MUX_RATIO, self.height - 1,
            _SET_COM_OUT_DIR | 0x08,
            _SET_DISP_OFFSET, 0x00,
            _SET_COM_PIN_CFG, 0x02 if self.width > 2 * self.height else 0x12,
            _SET_DISP_CLK_DIV, 0x80,
            _SET_PRECHARGE, 0x22 if self.external_vcc else 0xF1,
            _SET_VCOM_DESEL, 0x30,
            _SET_CONTRAST, 0xFF,
            _SET_ENTIRE_ON,
            _SET_NORM_INV,
            _SET_IREF_SELECT, 0x30,
            _SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
            _SET_DISP | 0x01,
        ):
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(_SET_DISP)

    def poweron(self):
        self.write_cmd(_SET_DISP | 0x01)

    def contrast(self, contrast):
        self.write_cmd(_SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(_SET_NORM_INV | (invert & 1))

    def show(self):
        self.write_cmd(_SET_COL_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.width - 1)
        self.write_cmd(_SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_data(self.buffer)


class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        self.write_list = [b"\x40", None]
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.write_list[1] = buf
        try:
            self.i2c.writevto(self.addr, self.write_list)
        except (AttributeError, NotImplementedError):
            self.i2c.writeto(self.addr, b"\x40" + buf)


# ═════════════════════════════════════════════════════════════════════
# 2. MPU6050 6-AXIS IMU DRIVER
# ═════════════════════════════════════════════════════════════════════

_MPU_PWR_MGMT_1   = const(0x6B)
_MPU_ACCEL_XOUT_H = const(0x3B)
_MPU_GYRO_XOUT_H  = const(0x43)
_MPU_ACCEL_CONFIG  = const(0x1C)
_MPU_GYRO_CONFIG   = const(0x1B)
_MPU_WHO_AM_I      = const(0x75)

_ACCEL_SCALE = {0: 16384, 1: 8192, 2: 4096, 3: 2048}  # ±2g, ±4g, ±8g, ±16g
_GYRO_SCALE = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}


class MPU6050:
    def __init__(self, i2c, addr=0x68, accel_range=2, gyro_range=0):
        self.i2c = i2c
        self.addr = addr
        self.accel_scale = _ACCEL_SCALE[accel_range]
        self.gyro_scale = _GYRO_SCALE[gyro_range]
        self._buf6 = bytearray(6)

        try:
            who = self.i2c.readfrom_mem(self.addr, _MPU_WHO_AM_I, 1)[0]
        except OSError:
            raise RuntimeError("MPU6050 not responding at 0x{:02X}".format(self.addr))

        if who not in (0x68, 0x69, 0x70, 0x71, 0x72, 0x73, 0x75, 0x98):
            print("[WARN] MPU unexpected WHO_AM_I=0x{:02X}".format(who))

        # Wake up
        self.i2c.writeto_mem(self.addr, _MPU_PWR_MGMT_1, b'\x00')
        self.i2c.writeto_mem(self.addr, _MPU_ACCEL_CONFIG, bytes([accel_range << 3]))
        self.i2c.writeto_mem(self.addr, _MPU_GYRO_CONFIG, bytes([gyro_range << 3]))

    def _read_raw(self, reg):
        self.i2c.readfrom_mem_into(self.addr, reg, self._buf6)
        return struct.unpack(">hhh", self._buf6)

    def accel(self):
        raw = self._read_raw(_MPU_ACCEL_XOUT_H)
        s = self.accel_scale
        return (raw[0] / s, raw[1] / s, raw[2] / s)

    def gyro(self):
        raw = self._read_raw(_MPU_GYRO_XOUT_H)
        s = self.gyro_scale
        return (raw[0] / s, raw[1] / s, raw[2] / s)

    def accel_magnitude(self):
        ax, ay, az = self.accel()
        return (ax * ax + ay * ay + az * az) ** 0.5


# ═════════════════════════════════════════════════════════════════════
# 3. MLX90614 CONTACTLESS IR THERMOMETER DRIVER
# ═════════════════════════════════════════════════════════════════════

_MLX_REG_AMBIENT = const(0x06)
_MLX_REG_OBJECT  = const(0x07)
CORE_TEMP_OFFSET = 3.0


class MLX90614:
    def __init__(self, i2c, addr=0x5A):
        self.i2c = i2c
        self.addr = addr
        try:
            raw = self._read_temp_raw(_MLX_REG_AMBIENT)
            if raw == 0 or raw == 0x7FFF:
                raise RuntimeError("MLX90614 bus stuck")
        except Exception:
            raise RuntimeError("MLX90614 not responding at 0x{:02X}".format(addr))

    def _read_temp_raw(self, reg, retries=3):
        last_err = None
        for _ in range(retries):
            try:
                try:
                    data = self.i2c.readfrom_mem(self.addr, reg, 3)
                except OSError:
                    data = self.i2c.readfrom_mem(self.addr, reg, 2)
                if (data[1] & 0x80) != 0:
                    raise RuntimeError("MLX error bit set")
                return data[0] | ((data[1] & 0x7F) << 8)
            except Exception as e:
                last_err = e
                time.sleep_ms(5)
        raise last_err if last_err else RuntimeError("MLX read failed")

    def _raw_to_c(self, raw):
        return (raw * 0.02) - 273.15

    def ambient_temp(self):
        return round(self._raw_to_c(self._read_temp_raw(_MLX_REG_AMBIENT)), 1)

    def object_temp(self):
        return round(self._raw_to_c(self._read_temp_raw(_MLX_REG_OBJECT)), 1)

    def is_skin_detected(self):
        try:
            return self.object_temp() >= 28.0
        except Exception:
            return False

    def body_temp(self):
        t = self.object_temp()
        if t < -20 or t > 70:
            raise ValueError("Unrealistic temp: {:.1f}C".format(t))
        if t >= 28.0:
            return round(t + CORE_TEMP_OFFSET, 1)
        return round(t, 1)


# ═════════════════════════════════════════════════════════════════════
# 4. MAX30102 PULSE OXIMETER & HEART RATE DRIVER
# ═════════════════════════════════════════════════════════════════════

_MAX_FIFO_WR_PTR   = const(0x04)
_MAX_OVF_COUNTER   = const(0x05)
_MAX_FIFO_RD_PTR   = const(0x06)
_MAX_FIFO_DATA     = const(0x07)
_MAX_FIFO_CONFIG   = const(0x08)
_MAX_MODE_CONFIG   = const(0x09)
_MAX_SPO2_CONFIG   = const(0x0A)
_MAX_LED1_PA       = const(0x0C)
_MAX_LED2_PA       = const(0x0D)
_MAX_INTR_ENABLE_1 = const(0x02)
_MAX_INTR_ENABLE_2 = const(0x03)
_MAX_INTR_STATUS_1 = const(0x00)
_MAX_PART_ID       = const(0xFF)


class MAX30102:
    def __init__(self, i2c, addr=0x57):
        self.i2c = i2c
        self.addr = addr

        part_id = self._read_reg(_MAX_PART_ID)
        if part_id != 0x15:
            raise RuntimeError("MAX30102 not found (PART_ID=0x{:02X})".format(part_id))

        self._setup()
        self._ir_buffer = []
        self._red_buffer = []
        self._bpm = 0
        self._spo2 = 0
        self._finger_detected = False

    def _read_reg(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def _write_reg(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _setup(self):
        # Reset
        self._write_reg(_MAX_MODE_CONFIG, 0x40)
        time.sleep_ms(100)

        # Clear FIFO
        self._write_reg(_MAX_FIFO_WR_PTR, 0x00)
        self._write_reg(_MAX_OVF_COUNTER, 0x00)
        self._write_reg(_MAX_FIFO_RD_PTR, 0x00)

        # FIFO config: 4 sample avg, rollover enabled
        self._write_reg(_MAX_FIFO_CONFIG, 0x4F)

        # SpO2 mode (Red + IR)
        self._write_reg(_MAX_MODE_CONFIG, 0x03)

        # SpO2 config: 4096nA, 100 SPS, 411us pulse width
        self._write_reg(_MAX_SPO2_CONFIG, 0x27)

        # Boosted LED amplitude (~10.6mA for bare finger reflection)
        self._write_reg(_MAX_LED1_PA, 0x34)
        self._write_reg(_MAX_LED2_PA, 0x34)

        self._write_reg(_MAX_INTR_ENABLE_1, 0xC0)
        self._write_reg(_MAX_INTR_ENABLE_2, 0x00)
        self._read_reg(_MAX_INTR_STATUS_1)

    def _read_fifo_single(self):
        try:
            data = self.i2c.readfrom_mem(self.addr, _MAX_FIFO_DATA, 6)
            red = ((data[0] << 16) | (data[1] << 8) | data[2]) & 0x03FFFF
            ir  = ((data[3] << 16) | (data[4] << 8) | data[5]) & 0x03FFFF
            return red, ir
        except OSError:
            return 0, 0

    def drain_fifo(self):
        try:
            wr = self._read_reg(_MAX_FIFO_WR_PTR)
            rd = self._read_reg(_MAX_FIFO_RD_PTR)
        except OSError:
            return 0

        num_samples = (wr - rd) & 0x1F
        if num_samples == 0:
            return 0

        try:
            raw = self.i2c.readfrom_mem(self.addr, _MAX_FIFO_DATA, num_samples * 6)
        except OSError:
            return 0

        processed = 0
        for i in range(0, len(raw), 6):
            if i + 6 > len(raw):
                break
            red = ((raw[i] << 16) | (raw[i+1] << 8) | raw[i+2]) & 0x03FFFF
            ir  = ((raw[i+3] << 16) | (raw[i+4] << 8) | raw[i+5]) & 0x03FFFF

            # Finger threshold
            is_finger = (ir > 35000)

            if not is_finger:
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
                processed += 1

        if self._finger_detected and len(self._ir_buffer) >= 50:
            self._calc_bpm()
            self._calc_spo2()

        return processed

    def _calc_bpm(self):
        buf = self._ir_buffer
        n = len(buf)
        if n < 50:
            return

        mean = sum(buf) // n
        max_v = max(buf)
        min_v = min(buf)

        # Pulse amplitude floor
        if (max_v - min_v) < 400:
            return

        threshold = mean + int((max_v - mean) * 0.35)

        # Refractory period blanking (min 7 samples = ~214 BPM at 25 SPS)
        peaks = []
        last_peak = -999
        for i in range(2, n - 2):
            if (i - last_peak) < 7:
                continue
            val = buf[i]
            if val > threshold and val > buf[i-1] and val > buf[i+1] and val >= buf[i-2] and val >= buf[i+2]:
                peaks.append(i)
                last_peak = i

        if len(peaks) >= 2:
            intervals = [peaks[i+1] - peaks[i] for i in range(len(peaks) - 1)]
            valid = [iv for iv in intervals if 7 <= iv <= 38]
            if valid:
                avg_iv = sum(valid) / len(valid)
                calc = int(60 * 25 / avg_iv)
                calc = max(40, min(200, calc))
                if self._bpm == 0:
                    self._bpm = calc
                else:
                    self._bpm = int(0.7 * self._bpm + 0.3 * calc)

    def _calc_spo2(self):
        rb = self._red_buffer[-50:]
        ib = self._ir_buffer[-50:]

        rdc = sum(rb) / len(rb)
        idc = sum(ib) / len(ib)
        if rdc < 1000 or idc < 1000:
            return

        rac = (sum((r - rdc) ** 2 for r in rb) / len(rb)) ** 0.5
        iac = (sum((i - idc) ** 2 for i in ib) / len(ib)) ** 0.5
        if rac < 15 or iac < 15:
            return

        ratio = (rac / rdc) / (iac / idc)
        calc_sp = int(110 - 25 * ratio)
        calc_sp = max(75, min(100, calc_sp))

        if self._spo2 == 0:
            self._spo2 = calc_sp
        else:
            self._spo2 = int(0.8 * self._spo2 + 0.2 * calc_sp)

    @property
    def bpm(self):
        return self._bpm

    @property
    def spo2(self):
        return self._spo2

    @property
    def finger_on(self):
        return self._finger_detected


# ═════════════════════════════════════════════════════════════════════
# 5. CORE APPLICATION, TELEMETRY & HARDWARE ORCHESTRATION
# ═════════════════════════════════════════════════════════════════════

PIN_SDA   = 8
PIN_SCL   = 9
PIN_SOS   = 5
PIN_MOTOR = 18

SCREEN_W = 128
SCREEN_H = 64

# Clinical Alert Thresholds
BPM_LOW        = 50
BPM_HIGH       = 120
SPO2_LOW       = 90     # %
TEMP_HIGH      = 38.5   # °C
FALL_THRESHOLD = 2.5    # g

SUBTICK_MS       = 40   # 25 Hz fast polling
TICKS_PER_SECOND = 25   # 25 * 40ms = 1000ms
MOTOR_PULSE_MS   = 200


def init_hardware():
    print("[BOOT] IoMT RPM Node v1.0 — MicroPython All-In-One")
    print("[BOOT] Initializing SoftI2C bus on SDA={}, SCL={}...".format(PIN_SDA, PIN_SCL))

    i2c = SoftI2C(sda=Pin(PIN_SDA, Pin.PULL_UP), scl=Pin(PIN_SCL, Pin.PULL_UP), freq=100_000)

    devices = i2c.scan()
    if len(devices) > 10:
        print("[FAULT] I2C Bus Error: 112 phantom devices! Bus shorted to GND or pulled LOW.")
        devices = []
    else:
        print("[I2C] Found {} device(s): {}".format(len(devices), [hex(d) for d in devices]))

    oled = None
    imu = None
    hr_sensor = None
    temp_sensor = None

    # SSD1306 (0x3C)
    try:
        oled = SSD1306_I2C(SCREEN_W, SCREEN_H, i2c, addr=0x3C)
        print("[OK] SSD1306 OLED initialized (0x3C)")
    except Exception as e:
        print("[FAIL] SSD1306: {}".format(e))

    # MPU6050 (0x68)
    try:
        imu = MPU6050(i2c, addr=0x68, accel_range=2)
        print("[OK] MPU6050 IMU initialized (0x68, ±8g)")
    except Exception as e:
        print("[FAIL] MPU6050: {}".format(e))

    # MAX30102 (0x57)
    try:
        hr_sensor = MAX30102(i2c, addr=0x57)
        print("[OK] MAX30102 Pulse Oximeter initialized (0x57)")
    except Exception as e:
        print("[FAIL] MAX30102: {}".format(e))

    # MLX90614 (0x5A)
    try:
        temp_sensor = MLX90614(i2c, addr=0x5A)
        print("[OK] MLX90614 IR Temp initialized (0x5A)")
    except Exception as e:
        print("[FAIL] MLX90614: {}".format(e))

    sos_btn = Pin(PIN_SOS, Pin.IN, Pin.PULL_UP)
    motor   = Pin(PIN_MOTOR, Pin.OUT, value=0)

    return oled, imu, hr_sensor, temp_sensor, sos_btn, motor


def show_boot_screen(oled):
    if oled is None:
        return
    oled.fill(0)
    oled.rect(0, 0, SCREEN_W, SCREEN_H, 1)
    oled.text("IoMT RPM v1.0", 8, 6, 1)
    oled.hline(0, 16, SCREEN_W, 1)
    oled.text("SYSTEM: ONLINE", 8, 24, 1)
    oled.text("Sensors: ALL-IN-1", 8, 36, 1)
    oled.text("Status: INIT...", 8, 48, 1)
    try:
        oled.show()
    except OSError:
        pass


def show_vitals_screen(oled, bpm, spo2, temp_c, finger_on, fall, sos, alert):
    if oled is None:
        return

    oled.fill(0)

    # Top Alert Bar
    if alert:
        oled.fill_rect(0, 0, SCREEN_W, 12, 1)
        oled.text("!! ALERT !!", 20, 2, 0)
    else:
        oled.text("IoMT RPM Node", 12, 2, 1)

    oled.hline(0, 13, SCREEN_W, 1)

    # Vitals
    bpm_str = str(bpm) if (bpm and bpm > 0) else "--"
    spo2_str = "{}%".format(spo2) if (spo2 and spo2 > 0) else "--"
    oled.text("HR : {}".format(bpm_str), 4, 18, 1)
    oled.text("SpO2: {}".format(spo2_str), 64, 18, 1)

    temp_str = "{:.1f}C".format(temp_c) if (temp_c is not None and temp_c > 0) else "--"
    oled.text("Temp: {}".format(temp_str), 4, 30, 1)

    # Status
    status_y = 44
    if fall:
        oled.text("FALL DETECTED!", 4, status_y, 1)
    elif sos:
        oled.text("SOS ACTIVATED!", 4, status_y, 1)
    elif not finger_on:
        oled.text("Attach Finger...", 4, status_y, 1)
    else:
        oled.text("Vitals Active", 4, status_y, 1)

    # Bottom Uptime
    oled.hline(0, 56, SCREEN_W, 1)
    uptime = time.ticks_ms() // 1000
    oled.text("Up:{}s".format(uptime), 4, 58, 1)

    try:
        oled.show()
    except OSError:
        pass


def check_alerts(bpm, spo2, temp_c, fall, sos):
    if sos or fall:
        return True
    if bpm is not None and bpm > 0 and (bpm < BPM_LOW or bpm > BPM_HIGH):
        return True
    if spo2 is not None and spo2 > 0 and spo2 < SPO2_LOW:
        return True
    if temp_c is not None and temp_c > TEMP_HIGH:
        return True
    return False


def pulse_motor(motor, on_ms=200):
    motor.value(1)
    time.sleep_ms(on_ms)
    motor.value(0)


def main():
    oled, imu, hr_sensor, temp_sensor, sos_btn, motor = init_hardware()

    show_boot_screen(oled)
    print("[BOOT] Boot splash screen displayed — starting in 3s...")
    time.sleep(3)

    print("[RUN] Entering main monitoring loop (25 Hz fast IMU/PPG tick)")
    print("[RUN] Streaming JSON telemetry on UART at 115200 baud\n")

    bpm = None
    spo2 = None
    finger_on = False
    temp_c = None
    fall_detected = False
    sos_active = False
    alert_active = False
    boot_time = time.ticks_ms()

    # Motor non-blocking timer & state
    motor_off_time = 0
    fall_cooldown = 0
    subtick_count = 0

    while True:
        tick_start = time.ticks_ms()

        # 1. Fast MPU6050 Fall Detection (25 Hz)
        # Edge-triggered: prints once per impact event, then maintains cooldown
        if imu:
            try:
                accel_mag = imu.accel_magnitude()
                if accel_mag > FALL_THRESHOLD:
                    if not fall_detected:
                        print("[ALERT] Fall impact detected! Peak accel = {:.2f}g".format(accel_mag))
                        fall_detected = True
                    fall_cooldown = time.ticks_add(tick_start, 10_000)
            except Exception:
                pass

        # 2. Fast MAX30102 FIFO Drain
        if hr_sensor:
            try:
                hr_sensor.drain_fifo()
            except Exception:
                pass

        # 3. Fast SOS Button Sample
        sos_active = (sos_btn.value() == 0)

        # 4. Non-blocking Vibration Motor Timer
        # Automatically turns off motor without stalling the 25Hz IMU loop
        if motor_off_time != 0 and time.ticks_diff(tick_start, motor_off_time) >= 0:
            motor.value(0)
            motor_off_time = 0

        # Auto-clear fall flag after 10s cooldown expires
        if fall_detected and time.ticks_diff(tick_start, fall_cooldown) > 0:
            fall_detected = False

        subtick_count += 1

        # 1-Second Periodic Update (every 25 subticks)
        if subtick_count >= TICKS_PER_SECOND:
            subtick_count = 0

            # A. Update PPG Heart Rate & SpO2
            if hr_sensor:
                finger_on = hr_sensor.finger_on
                if finger_on:
                    bpm = hr_sensor.bpm if hr_sensor.bpm > 0 else None
                    spo2 = hr_sensor.spo2 if hr_sensor.spo2 > 0 else None
                else:
                    bpm = None
                    spo2 = None
            else:
                finger_on = False
                bpm = None
                spo2 = None

            # B. Read MLX90614 IR Temperature
            if temp_sensor:
                try:
                    temp_c = temp_sensor.body_temp()
                except Exception:
                    temp_c = None
            else:
                temp_c = None

            # C. Check Clinical Alerts
            alert_active = check_alerts(bpm, spo2, temp_c, fall_detected, sos_active)

            # D. Trigger Non-Blocking Vibration Motor Pulse
            if alert_active and motor_off_time == 0:
                motor.value(1)
                motor_off_time = time.ticks_add(tick_start, MOTOR_PULSE_MS)

            # E. Update OLED Display
            try:
                show_vitals_screen(oled, bpm, spo2, temp_c, finger_on, fall_detected, sos_active, alert_active)
            except OSError:
                pass

            # F. Stream JSON Telemetry
            uptime_s = time.ticks_diff(time.ticks_ms(), boot_time) // 1000
            telemetry = {
                "bpm": bpm,
                "spo2": spo2,
                "finger": finger_on,
                "temp_c": temp_c,
                "fall": fall_detected,
                "sos": sos_active,
                "alert": alert_active,
                "uptime_s": uptime_s,
            }
            print(json.dumps(telemetry))

        # Pace loop to 40ms (25 Hz)
        elapsed = time.ticks_diff(time.ticks_ms(), tick_start)
        delay = SUBTICK_MS - elapsed
        if delay > 0:
            time.sleep_ms(delay)


if __name__ == "__main__":
    main()
