"""
IoMT Remote Patient Monitoring (RPM) Node — MicroPython Main Application
ESP32-S3 DevKitC-1

Full real-hardware sensor suite:
  - MAX30102  (I2C 0x57)  Heart Rate + SpO₂
  - MLX90614  (I2C 0x5A)  Skin Temperature
  - MPU6050   (I2C 0x68)  Fall Detection (IMU)
  - SSD1306   (I2C 0x3C)  128×64 OLED Display
  - Pushbutton (GPIO 5)   SOS Emergency Button
  - Vibration Motor (GPIO 18) via MOSFET driver

Pin mapping (matches wiring-guide.html):
  GPIO 8  → I2C SDA  (shared bus: MAX30102, MLX90614, MPU6050, SSD1306)
  GPIO 9  → I2C SCL
  GPIO 2  → MAX30102 INT (optional)
  GPIO 5  → SOS pushbutton (INPUT_PULLUP, active LOW)
  GPIO 18 → Vibration motor (OUTPUT, via MOSFET gate)
"""

import json
import time
from machine import Pin, SoftI2C

# ── Local driver imports ─────────────────────────────────────────────
from ssd1306 import SSD1306_I2C
from mpu6050 import MPU6050
from max30102 import MAX30102
from mlx90614 import MLX90614

# ── Pin Definitions ──────────────────────────────────────────────────
PIN_SDA   = 8
PIN_SCL   = 9
PIN_SOS   = 5
PIN_MOTOR = 18

# ── Display Constants ────────────────────────────────────────────────
SCREEN_W = 128
SCREEN_H = 64

# ── Clinical Alert Thresholds ────────────────────────────────────────
BPM_LOW        = 50
BPM_HIGH       = 120
SPO2_LOW       = 90     # % — below this is hypoxemia
TEMP_HIGH      = 38.5   # °C — fever threshold (estimated core)
FALL_THRESHOLD = 2.5    # g — acceleration spike indicating fall

# ── Timing ───────────────────────────────────────────────────────────
SUBTICK_MS       = 40     # 25Hz fast loop for IMU & MAX30102 FIFO
TICKS_PER_SECOND = 25     # 25 * 40ms = 1000ms
MOTOR_PULSE_MS   = 200    # Vibration pulse ON time


def init_hardware():
    """Initialize I2C bus, sensors, display, button, and motor on real hardware."""

    print("[BOOT] IoMT RPM Node v1.0 — MicroPython (Real Hardware)")
    print("[BOOT] Initializing I2C bus on SDA={}, SCL={}...".format(PIN_SDA, PIN_SCL))

    # Use SoftI2C with internal pull-ups for maximum compatibility with MLX90614 clock stretching and multi-device bus loading
    i2c = SoftI2C(sda=Pin(PIN_SDA, Pin.PULL_UP), scl=Pin(PIN_SCL, Pin.PULL_UP), freq=100_000)
    print("[I2C] SoftI2C bus initialized with internal pull-ups on SDA={}, SCL={}".format(PIN_SDA, PIN_SCL))

    # Scan I2C bus
    devices = i2c.scan()
    if len(devices) > 10:
        print("[FAULT] I2C Bus Error: 112 phantom devices! SDA is shorted to GND or pulled LOW.")
        devices = []
    else:
        print("[I2C] Found {} device(s): {}".format(
            len(devices), [hex(d) for d in devices]
        ))

    # ── Initialize sensors (with graceful error handling) ────────
    oled = None
    imu = None
    hr_sensor = None
    temp_sensor = None

    # SSD1306 OLED Display (0x3C)
    try:
        oled = SSD1306_I2C(SCREEN_W, SCREEN_H, i2c, addr=0x3C)
        print("[OK] SSD1306 OLED initialized (0x3C)")
    except Exception as e:
        print("[FAIL] SSD1306: {}".format(e))

    # MPU6050 IMU (0x68) - accel_range=2 (±8g) for fall impact detection
    try:
        imu = MPU6050(i2c, addr=0x68, accel_range=2)
        print("[OK] MPU6050 IMU initialized (0x68, ±8g)")
    except Exception as e:
        print("[FAIL] MPU6050: {}".format(e))

    # MAX30102 Pulse Oximeter (0x57)
    try:
        hr_sensor = MAX30102(i2c, addr=0x57)
        print("[OK] MAX30102 Pulse Oximeter initialized (0x57)")
    except Exception as e:
        print("[FAIL] MAX30102: {}".format(e))

    # MLX90614 IR Temperature (0x5A)
    try:
        temp_sensor = MLX90614(i2c, addr=0x5A)
        print("[OK] MLX90614 IR Temp initialized (0x5A)")
    except Exception as e:
        print("[FAIL] MLX90614: {}".format(e))

    # ── GPIO ─────────────────────────────────────────────────────
    sos_btn = Pin(PIN_SOS, Pin.IN, Pin.PULL_UP)
    motor   = Pin(PIN_MOTOR, Pin.OUT, value=0)

    return oled, imu, hr_sensor, temp_sensor, sos_btn, motor


def show_boot_screen(oled):
    """Display boot screen on OLED."""
    if oled is None:
        return

    oled.fill(0)
    oled.rect(0, 0, SCREEN_W, SCREEN_H, 1)
    oled.text("IoMT RPM v1.0", 8, 6, 1)
    oled.hline(0, 16, SCREEN_W, 1)
    oled.text("SYSTEM: ONLINE", 8, 24, 1)
    oled.text("Sensors: REAL", 8, 36, 1)
    oled.text("Status: INIT...", 8, 48, 1)

    try:
        oled.show()
    except OSError:
        pass


def show_vitals_screen(oled, bpm, spo2, temp_c, finger_on, fall, sos, alert):
    """Update the OLED with live real-sensor vitals dashboard."""
    if oled is None:
        return

    oled.fill(0)

    # Top bar with alert indicator
    if alert:
        oled.fill_rect(0, 0, SCREEN_W, 12, 1)
        oled.text("!! ALERT !!", 20, 2, 0)  # Inverted text
    else:
        oled.text("IoMT RPM Node", 12, 2, 1)

    oled.hline(0, 13, SCREEN_W, 1)

    # Vital signs (show real values or '--' when no finger / reading)
    bpm_str = str(bpm) if (bpm and bpm > 0) else "--"
    spo2_str = "{}%".format(spo2) if (spo2 and spo2 > 0) else "--"
    oled.text("HR : {}".format(bpm_str), 4, 18, 1)
    oled.text("SpO2: {}".format(spo2_str), 64, 18, 1)

    temp_str = "{:.1f}C".format(temp_c) if (temp_c is not None and temp_c > 0) else "--"
    oled.text("Temp: {}".format(temp_str), 4, 30, 1)

    # Clinical status / alerts
    status_y = 44
    if fall:
        oled.text("FALL DETECTED!", 4, status_y, 1)
    elif sos:
        oled.text("SOS ACTIVATED!", 4, status_y, 1)
    elif not finger_on:
        oled.text("Place Finger...", 4, status_y, 1)
    else:
        oled.text("Vitals Active", 4, status_y, 1)

    # Uptime bar at bottom
    oled.hline(0, 56, SCREEN_W, 1)
    uptime = time.ticks_ms() // 1000
    oled.text("Up:{}s".format(uptime), 4, 58, 1)

    try:
        oled.show()
    except OSError:
        pass


def check_alerts(bpm, spo2, temp_c, fall, sos):
    """Check if any clinical alert condition is active."""
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
    """Pulse the vibration motor for haptic alert."""
    motor.value(1)
    time.sleep_ms(on_ms)
    motor.value(0)


# ═════════════════════════════════════════════════════════════════════
# Main entry point
# ═════════════════════════════════════════════════════════════════════

def main():
    oled, imu, hr_sensor, temp_sensor, sos_btn, motor = init_hardware()

    # Show boot splash screen for 3 seconds
    show_boot_screen(oled)
    print("[BOOT] Boot screen displayed — starting in 3s...")
    time.sleep(3)

    print("[RUN] Entering main monitoring loop")
    print("[RUN] IMU fall detection sampling at 25 Hz")
    print("[RUN] JSON telemetry streamed on UART at 115200 baud")
    print()

    # ── State ────────────────────────────────────────────────────
    bpm = None
    spo2 = None
    finger_on = False
    temp_c = None
    fall_detected = False
    sos_active = False
    alert_active = False
    boot_time = time.ticks_ms()

    # Fall detection & timing state
    fall_cooldown = 0
    subtick_count = 0

    while True:
        tick_start = time.ticks_ms()

        # ── 1. Fast Task: High-Frequency MPU6050 Fall Detection (25Hz) ───
        if imu:
            try:
                accel_mag = imu.accel_magnitude()
                if accel_mag > FALL_THRESHOLD:
                    fall_detected = True
                    fall_cooldown = time.ticks_add(tick_start, 10_000)  # 10s alert hold
                    print("[ALERT] Fall impact detected! Peak accel = {:.2f}g".format(accel_mag))
            except Exception:
                pass

        # ── 2. Fast Task: Drain MAX30102 FIFO ────────────────────
        if hr_sensor:
            try:
                hr_sensor.drain_fifo()
            except Exception:
                pass

        # ── 3. Fast Task: Read SOS Button (Active LOW) ───────────
        if sos_btn.value() == 0:
            sos_active = True
        else:
            sos_active = False

        # Auto-clear fall flag after cooldown expires
        if fall_detected and time.ticks_diff(tick_start, fall_cooldown) > 0:
            fall_detected = False

        subtick_count += 1

        # ── 1-Second Periodic Processing (Every 25 subticks) ─────
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

            # B. Read MLX90614 Contactless IR Temperature
            if temp_sensor:
                try:
                    temp_c = temp_sensor.body_temp()
                except Exception:
                    temp_c = None
            else:
                temp_c = None

            # C. Evaluate Clinical Alerts
            alert_active = check_alerts(bpm, spo2, temp_c, fall_detected, sos_active)

            # D. Fire Vibration Motor Alert
            if alert_active:
                pulse_motor(motor, MOTOR_PULSE_MS)

            # E. Render OLED Display
            try:
                show_vitals_screen(oled, bpm, spo2, temp_c, finger_on, fall_detected, sos_active, alert_active)
            except OSError:
                pass

            # F. Structured JSON Telemetry over UART
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

        # Pace fast tick to 40ms (25 Hz)
        elapsed = time.ticks_diff(time.ticks_ms(), tick_start)
        delay = SUBTICK_MS - elapsed
        if delay > 0:
            time.sleep_ms(delay)


# Run
main()
