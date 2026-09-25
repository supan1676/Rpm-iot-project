"""
IoMT Remote Patient Monitoring (RPM) Node — MicroPython, ESP32-S3
v2: Ported fall-detection state machine + alert screen + step counter.

Features:
  - Two-stage FREE_FALL -> IMPACT -> FALL_DETECTED state machine with timeout disarm
  - Full-screen cancellable alert with live countdown and warning icon
  - Context-aware SOS button: triggers alert when idle, cancels alert when active
  - MPU6050 step counter using moving-average acceleration magnitude filter
  - Animated pulsing heart icon and custom geometric drawing primitives for framebuf
  - SSD1306 128x64 OLED dashboard
  - MAX30102 PPG pulse oximeter with bulk FIFO draining and refractory blanking
  - MLX90614 Contactless IR body thermometer with SMBus retry logic
  - Non-blocking haptic vibration alerts
  - Structured serial JSON telemetry at 115200 baud

Wiring (ESP32-S3):
  GPIO 8  -> I2C SDA (OLED, MPU6050, MLX90614, MAX30102)
  GPIO 9  -> I2C SCL (OLED, MPU6050, MLX90614, MAX30102)
  GPIO 5  -> SOS Pushbutton (INPUT_PULLUP, active LOW)
  GPIO 18 -> Vibration motor (via MOSFET gate)
  3.3V    -> Sensor VCC / VIN
  GND     -> Sensor GND (and MPU6050 AD0)

Hardware Tip:
  External 4.7kΩ pull-up resistors on SDA and SCL to 3.3V are strongly
  recommended for 4-device breadboard buses to ensure crisp I2C edges.

Disclaimer:
  This firmware is an IoMT engineering prototype / academic demonstrator.
  Empirical vitals calculations are standard research approximations
  and are not intended for certified clinical or medical diagnosis.
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

    def contrast(self, contrast):
        self.write_cmd(_SET_CONTRAST)
        self.write_cmd(contrast)

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
# 2. DRAWING HELPERS (Geometric primitives for MicroPython framebuf)
# ═════════════════════════════════════════════════════════════════════

def fill_circle(fb, cx, cy, r):
    for dy in range(-r, r + 1):
        dx = int((r * r - dy * dy) ** 0.5)
        fb.hline(cx - dx, cy + dy, dx * 2 + 1, 1)


def fill_triangle(fb, x0, y0, x1, y1, x2, y2):
    pts = sorted([(x0, y0), (x1, y1), (x2, y2)], key=lambda p: p[1])
    (x0, y0), (x1, y1), (x2, y2) = pts

    def interp(y, xa, ya, xb, yb):
        if yb == ya:
            return xa
        return xa + (xb - xa) * (y - ya) // (yb - ya)

    for y in range(y0, y2 + 1):
        xa = interp(y, x0, y0, x2, y2)
        xb = interp(y, x0, y0, x1, y1) if y < y1 else interp(y, x1, y1, x2, y2)
        if xa > xb:
            xa, xb = xb, xa
        fb.hline(int(xa), y, int(xb - xa) + 1, 1)


def draw_heart(fb, x, y, pulse):
    r = 4 if pulse else 3
    fill_circle(fb, x, y, r)
    fill_circle(fb, x + r * 2, y, r)
    fill_triangle(fb, x - r, y, x + r * 3, y, x + r, y + r * 3)


def draw_warning_triangle(fb, cx, top_y):
    fb.line(cx, top_y, cx - 14, top_y + 24, 1)
    fb.line(cx, top_y, cx + 14, top_y + 24, 1)
    fb.line(cx - 14, top_y + 24, cx + 14, top_y + 24, 1)
    fb.vline(cx, top_y + 6, 10, 1)
    fb.pixel(cx, top_y + 20, 1)
    fb.pixel(cx - 1, top_y + 20, 1)


# ═════════════════════════════════════════════════════════════════════
# 3. MPU6050 6-AXIS IMU DRIVER
# ═════════════════════════════════════════════════════════════════════

_MPU_PWR_MGMT_1   = const(0x6B)
_MPU_ACCEL_XOUT_H = const(0x3B)
_MPU_ACCEL_CONFIG = const(0x1C)
_MPU_GYRO_CONFIG  = const(0x1B)
_MPU_WHO_AM_I     = const(0x75)

_ACCEL_SCALE = {0: 16384, 1: 8192, 2: 4096, 3: 2048}


class MPU6050:
    def __init__(self, i2c, addr=0x68, accel_range=2, gyro_range=0):
        self.i2c = i2c
        self.addr = addr
        self.accel_scale = _ACCEL_SCALE[accel_range]
        self._buf6 = bytearray(6)

        try:
            who = self.i2c.readfrom_mem(self.addr, _MPU_WHO_AM_I, 1)[0]
        except OSError:
            raise RuntimeError("MPU6050 not responding at 0x{:02X}".format(self.addr))
        if who not in (0x68, 0x69, 0x70, 0x71, 0x72, 0x73, 0x75, 0x98):
            print("[WARN] MPU unexpected WHO_AM_I=0x{:02X}".format(who))

        self.i2c.writeto_mem(self.addr, _MPU_PWR_MGMT_1, b'\x00')
        self.i2c.writeto_mem(self.addr, _MPU_ACCEL_CONFIG, bytes([accel_range << 3]))
        self.i2c.writeto_mem(self.addr, _MPU_GYRO_CONFIG, bytes([gyro_range << 3]))

    def accel(self):
        self.i2c.readfrom_mem_into(self.addr, _MPU_ACCEL_XOUT_H, self._buf6)
        raw = struct.unpack(">hhh", self._buf6)
        s = self.accel_scale
        return (raw[0] / s, raw[1] / s, raw[2] / s)

    def accel_magnitude(self):
        ax, ay, az = self.accel()
        return (ax * ax + ay * ay + az * az) ** 0.5


# ═════════════════════════════════════════════════════════════════════
# 4. MLX90614 CONTACTLESS IR THERMOMETER DRIVER
# ═════════════════════════════════════════════════════════════════════

_MLX_REG_OBJECT = const(0x07)
CORE_TEMP_OFFSET = 3.0


class MLX90614:
    def __init__(self, i2c, addr=0x5A):
        self.i2c = i2c
        self.addr = addr
        try:
            self._read_temp_raw(_MLX_REG_OBJECT)
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

    def object_temp(self):
        raw = self._read_temp_raw(_MLX_REG_OBJECT)
        return round((raw * 0.02) - 273.15, 1)

    def body_temp(self):
        t = self.object_temp()
        if t < -20 or t > 70:
            raise ValueError("Unrealistic temp: {:.1f}C".format(t))
        return round(t + CORE_TEMP_OFFSET, 1) if t >= 28.0 else t


# ═════════════════════════════════════════════════════════════════════
# 5. MAX30102 PULSE OXIMETER DRIVER
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
        self._write_reg(_MAX_MODE_CONFIG, 0x40)
        time.sleep_ms(100)
        self._write_reg(_MAX_FIFO_WR_PTR, 0x00)
        self._write_reg(_MAX_OVF_COUNTER, 0x00)
        self._write_reg(_MAX_FIFO_RD_PTR, 0x00)
        self._write_reg(_MAX_FIFO_CONFIG, 0x4F)
        self._write_reg(_MAX_MODE_CONFIG, 0x03)
        self._write_reg(_MAX_SPO2_CONFIG, 0x27)
        self._write_reg(_MAX_LED1_PA, 0x34)
        self._write_reg(_MAX_LED2_PA, 0x34)
        self._write_reg(_MAX_INTR_ENABLE_1, 0xC0)
        self._write_reg(_MAX_INTR_ENABLE_2, 0x00)
        self._read_reg(_MAX_INTR_STATUS_1)

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

        for i in range(0, len(raw) - 5, 6):
            red = ((raw[i] << 16) | (raw[i + 1] << 8) | raw[i + 2]) & 0x03FFFF
            ir = ((raw[i + 3] << 16) | (raw[i + 4] << 8) | raw[i + 5]) & 0x03FFFF
            if ir <= 35000:
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

        if self._finger_detected and len(self._ir_buffer) >= 50:
            self._calc_bpm()
            self._calc_spo2()
        return num_samples

    def _calc_bpm(self):
        buf = self._ir_buffer
        n = len(buf)
        mean = sum(buf) // n
        max_v, min_v = max(buf), min(buf)
        if (max_v - min_v) < 400:
            return
        threshold = mean + int((max_v - mean) * 0.35)
        peaks, last_peak = [], -999
        for i in range(2, n - 2):
            if (i - last_peak) < 7:
                continue
            v = buf[i]
            if v > threshold and v > buf[i-1] and v > buf[i+1] and v >= buf[i-2] and v >= buf[i+2]:
                peaks.append(i)
                last_peak = i
        if len(peaks) >= 2:
            intervals = [peaks[i+1] - peaks[i] for i in range(len(peaks) - 1)]
            valid = [iv for iv in intervals if 7 <= iv <= 38]
            if valid:
                calc = max(40, min(200, int(60 * 25 / (sum(valid) / len(valid)))))
                self._bpm = calc if self._bpm == 0 else int(0.7 * self._bpm + 0.3 * calc)

    def _calc_spo2(self):
        rb, ib = self._red_buffer[-50:], self._ir_buffer[-50:]
        rdc, idc = sum(rb) / len(rb), sum(ib) / len(ib)
        if rdc < 1000 or idc < 1000:
            return
        rac = (sum((r - rdc) ** 2 for r in rb) / len(rb)) ** 0.5
        iac = (sum((i - idc) ** 2 for i in ib) / len(ib)) ** 0.5
        if rac < 15 or iac < 15:
            return
        ratio = (rac / rdc) / (iac / idc)
        calc_sp = max(75, min(100, int(110 - 25 * ratio)))
        self._spo2 = calc_sp if self._spo2 == 0 else int(0.8 * self._spo2 + 0.2 * calc_sp)

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
# 6. CONFIGURATION & THRESHOLDS
# ═════════════════════════════════════════════════════════════════════

PIN_SDA, PIN_SCL, PIN_SOS, PIN_MOTOR = 8, 9, 5, 18
SCREEN_W, SCREEN_H = 128, 64

BPM_LOW, BPM_HIGH, SPO2_LOW, TEMP_HIGH = 50, 120, 90, 38.5

# Fall thresholds (converted from m/s^2 to g)
FREE_FALL_G         = 10.0 / 9.8   # ~1.02g — momentary dip below normal gravity
IMPACT_G            = 12.0 / 9.8   # ~1.22g — spike following the dip
FREE_FALL_WINDOW_MS = 120          # dip must be followed by impact within window
IMPACT_WAIT_MS      = 1000         # disarm if no spike follows within 1s

ALERT_TIMEOUT_MS = 20000           # 20 second countdown for emergency acknowledgement

STEP_THRESHOLD_G = 0.9
STEP_DELAY_MS    = 500
MAG_WINDOW       = 5

SUBTICK_MS = 40                    # 25 Hz fast tick


class FallState:
    MONITORING = 0
    IMPACT_WAIT = 1
    DETECTED = 2


# ═════════════════════════════════════════════════════════════════════
# 7. HARDWARE INITIALIZATION
# ═════════════════════════════════════════════════════════════════════

def init_hardware():
    print("[BOOT] IoMT RPM Node v2 — MicroPython All-In-One")
    i2c = SoftI2C(sda=Pin(PIN_SDA, Pin.PULL_UP), scl=Pin(PIN_SCL, Pin.PULL_UP), freq=100_000)
    devices = i2c.scan()
    print("[I2C] Found {} device(s): {}".format(len(devices), [hex(d) for d in devices]))

    oled = imu = hr_sensor = temp_sensor = None
    try:
        oled = SSD1306_I2C(SCREEN_W, SCREEN_H, i2c, addr=0x3C)
        print("[OK] SSD1306 OLED initialized (0x3C)")
    except Exception as e:
        print("[FAIL] SSD1306: {}".format(e))
    try:
        imu = MPU6050(i2c, addr=0x68, accel_range=2)
        print("[OK] MPU6050 IMU initialized (0x68, +/-8g)")
    except Exception as e:
        print("[FAIL] MPU6050: {}".format(e))
    try:
        hr_sensor = MAX30102(i2c, addr=0x57)
        print("[OK] MAX30102 initialized (0x57)")
    except Exception as e:
        print("[FAIL] MAX30102: {}".format(e))
    try:
        temp_sensor = MLX90614(i2c, addr=0x5A)
        print("[OK] MLX90614 initialized (0x5A)")
    except Exception as e:
        print("[FAIL] MLX90614: {}".format(e))

    sos_btn = Pin(PIN_SOS, Pin.IN, Pin.PULL_UP)
    motor = Pin(PIN_MOTOR, Pin.OUT, value=0)
    return oled, imu, hr_sensor, temp_sensor, sos_btn, motor


# ═════════════════════════════════════════════════════════════════════
# 8. SCREEN RENDERING
# ═════════════════════════════════════════════════════════════════════

def show_boot_screen(oled):
    if not oled:
        return
    oled.fill(0)
    oled.rect(0, 0, SCREEN_W, SCREEN_H, 1)
    oled.text("IoMT RPM v2", 12, 6, 1)
    oled.hline(0, 16, SCREEN_W, 1)
    oled.text("SYSTEM: ONLINE", 8, 26, 1)
    oled.text("Status: INIT...", 8, 40, 1)
    try:
        oled.show()
    except OSError:
        pass


def show_vitals_screen(oled, bpm, spo2, temp_c, finger_on, steps, pulse):
    if not oled:
        return
    oled.fill(0)
    draw_heart(oled, 8, 8, pulse)
    oled.text("IoMT RPM", 40, 4, 1)
    oled.hline(0, 16, SCREEN_W, 1)

    bpm_str = str(bpm) if bpm else "--"
    spo2_str = "{}%".format(spo2) if spo2 else "--"
    oled.text("HR:{}".format(bpm_str), 2, 22, 1)
    oled.text("SpO2:{}".format(spo2_str), 68, 22, 1)

    temp_str = "{:.1f}C".format(temp_c) if temp_c else "--"
    oled.text("Temp:{}".format(temp_str), 2, 34, 1)
    oled.text("Steps:{}".format(steps), 68, 34, 1)

    status = "Vitals Active" if finger_on else "Attach Finger.."
    oled.text(status, 2, 46, 1)

    oled.hline(0, 56, SCREEN_W, 1)
    oled.text("Up:{}s".format(time.ticks_ms() // 1000), 4, 58, 1)
    try:
        oled.show()
    except OSError:
        pass


def show_alert_screen(oled, reason, remaining_ms):
    if not oled:
        return
    oled.fill(0)
    oled.text("!! ALERT: {} !!".format(reason), 2, 2, 1)
    oled.hline(0, 12, SCREEN_W, 1)
    draw_warning_triangle(oled, 64, 18)
    oled.text("Press SOS button", 16, 46, 1)
    oled.text("to cancel  ({}s)".format(max(0, remaining_ms // 1000)), 12, 56, 1)
    try:
        oled.show()
    except OSError:
        pass


# ═════════════════════════════════════════════════════════════════════
# 9. MAIN ORCHESTRATION LOOP
# ═════════════════════════════════════════════════════════════════════

def main():
    oled, imu, hr_sensor, temp_sensor, sos_btn, motor = init_hardware()
    show_boot_screen(oled)
    time.sleep(3)
    print("[RUN] Entering main monitoring loop (25 Hz fast IMU/PPG tick)")
    print("[RUN] Streaming JSON telemetry on UART at 115200 baud\n")

    fall_state = FallState.MONITORING
    dip_start = 0

    alert_active = False
    alert_start = 0
    alert_reason = None
    last_alert_render = 0

    prev_btn = False
    last_btn_time = 0

    bpm = spo2 = temp_c = None
    finger_on = False
    step_count = 0
    last_step_time = 0
    last_filtered_mag = 0.0
    mag_buf = []

    pulse_on = False
    last_pulse_toggle = 0
    subtick = 0
    boot_time = time.ticks_ms()

    while True:
        tick = time.ticks_ms()

        # ── 1. Fast: Fall-Detection State Machine (25 Hz) ────────
        if imu:
            try:
                mag = imu.accel_magnitude()
            except Exception:
                mag = 1.0

            if fall_state == FallState.MONITORING and mag < FREE_FALL_G:
                dip_start = tick
                fall_state = FallState.IMPACT_WAIT
            elif fall_state == FallState.IMPACT_WAIT:
                dt = time.ticks_diff(tick, dip_start)
                if mag > IMPACT_G and dt > FREE_FALL_WINDOW_MS:
                    fall_state = FallState.DETECTED
                elif dt > IMPACT_WAIT_MS:
                    fall_state = FallState.MONITORING  # dip timed out without impact

            if fall_state == FallState.DETECTED and not alert_active:
                alert_active, alert_start, alert_reason = True, tick, "FALL"
                motor.value(1)
                print("[ALERT] Fall detected via 2-stage state machine!")
                fall_state = FallState.MONITORING

            # Step counting from accelerometer magnitude filter
            mag_buf.append(abs(mag - 1.0))
            if len(mag_buf) > MAG_WINDOW:
                mag_buf.pop(0)
            filtered = sum(mag_buf) / len(mag_buf)
            if (filtered > STEP_THRESHOLD_G and last_filtered_mag <= STEP_THRESHOLD_G
                    and time.ticks_diff(tick, last_step_time) > STEP_DELAY_MS):
                step_count += 1
                last_step_time = tick
            last_filtered_mag = filtered

        # ── 2. Fast: MAX30102 FIFO Drain ─────────────────────────
        if hr_sensor:
            try:
                hr_sensor.drain_fifo()
            except Exception:
                pass

        # ── 3. Fast: Dual-Action SOS Button ──────────────────────
        # Press while idle = trigger SOS alert
        # Press during active alert = acknowledge / cancel alert
        btn_down = (sos_btn.value() == 0)
        if btn_down and not prev_btn and time.ticks_diff(tick, last_btn_time) > 200:
            last_btn_time = tick
            if alert_active:
                alert_active = False
                motor.value(0)
                print("[ALERT] Cancelled by patient via SOS button")
            else:
                alert_active, alert_start, alert_reason = True, tick, "SOS"
                motor.value(1)
                print("[ALERT] SOS button pressed by patient")
        prev_btn = btn_down

        # ── 4. Fast: Alert Auto-Timeout ──────────────────────────
        if alert_active and time.ticks_diff(tick, alert_start) > ALERT_TIMEOUT_MS:
            alert_active = False
            motor.value(0)
            print("[ALERT] Timed out (20s unacknowledged)")

        # ── 5. Alert Screen Render (throttled to 250ms for fluid countdown without bus stall) ──
        if alert_active and time.ticks_diff(tick, last_alert_render) >= 250:
            last_alert_render = tick
            show_alert_screen(oled, alert_reason, ALERT_TIMEOUT_MS - time.ticks_diff(tick, alert_start))

        subtick += 1
        # ── 6. Periodic 1-Second Processing (Every 25 subticks) ───
        if subtick >= 25:
            subtick = 0

            # A. Update PPG Heart Rate & SpO2
            if hr_sensor:
                finger_on = hr_sensor.finger_on
                bpm = hr_sensor.bpm if finger_on and hr_sensor.bpm > 0 else None
                spo2 = hr_sensor.spo2 if finger_on and hr_sensor.spo2 > 0 else None
            else:
                finger_on, bpm, spo2 = False, None, None

            # B. Read MLX90614 Contactless IR Temperature
            if temp_sensor:
                try:
                    temp_c = temp_sensor.body_temp()
                except Exception:
                    temp_c = None

            # C. Check Clinical Vital Thresholds
            if not alert_active:
                out_of_range = (
                    (bpm is not None and (bpm < BPM_LOW or bpm > BPM_HIGH)) or
                    (spo2 is not None and spo2 < SPO2_LOW) or
                    (temp_c is not None and temp_c > TEMP_HIGH)
                )
                if out_of_range:
                    alert_active, alert_start, alert_reason = True, tick, "VITALS"
                    motor.value(1)
                    print("[ALERT] Clinical vitals out of range!")

            # D. Pulse heart icon toggle
            if time.ticks_diff(tick, last_pulse_toggle) > 600:
                pulse_on = not pulse_on
                last_pulse_toggle = tick

            # E. Normal Vitals Dashboard Render
            if not alert_active:
                show_vitals_screen(oled, bpm, spo2, temp_c, finger_on, step_count, pulse_on)

            # F. Structured JSON Telemetry over UART
            uptime_s = time.ticks_diff(time.ticks_ms(), boot_time) // 1000
            print(json.dumps({
                "bpm": bpm,
                "spo2": spo2,
                "finger": finger_on,
                "temp_c": temp_c,
                "fall": alert_reason == "FALL" if alert_active else False,
                "sos": alert_reason == "SOS" if alert_active else False,
                "alert": alert_active,
                "alert_reason": alert_reason if alert_active else None,
                "steps": step_count,
                "uptime_s": uptime_s,
            }))

        # Pace loop to 40ms (25 Hz)
        elapsed = time.ticks_diff(time.ticks_ms(), tick)
        delay = SUBTICK_MS - elapsed
        if delay > 0:
            time.sleep_ms(delay)


if __name__ == "__main__":
    main()
