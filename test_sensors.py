"""
Diagnostic script for IoMT RPM Node — Real Sensor & Hardware Tester.
Run this directly on ESP32 without typing into the REPL:
    python -m mpremote connect COM6 run test_sensors.py
"""

import time
from machine import Pin, SoftI2C

print("=" * 55)
print("       IoMT RPM Hardware & Sensor Diagnostic")
print("=" * 55)

PIN_SDA = 8
PIN_SCL = 9
PIN_SOS = 5
PIN_MOTOR = 18

print(f"\n[1] Testing I2C Bus on GPIO {PIN_SDA} (SDA) and GPIO {PIN_SCL} (SCL)...")
i2c = SoftI2C(sda=Pin(PIN_SDA, Pin.PULL_UP), scl=Pin(PIN_SCL, Pin.PULL_UP), freq=100000)

devices = i2c.scan()
hex_devs = [hex(d) for d in devices]
print(f"    Raw Scan Result: {hex_devs}")

EXPECTED = {
    0x3C: "SSD1306 OLED Display (128x64)",
    0x57: "MAX30102 Pulse Oximeter (PPG)",
    0x5A: "MLX90614 Contactless IR Thermometer",
    0x68: "MPU6050 6-Axis IMU (Accelerometer/Gyro)"
}

print("\n[2] Device Detection Summary:")
found_count = 0
for addr, name in EXPECTED.items():
    if addr in devices:
        print(f"    [ONLINE]  {hex(addr)} -> {name}")
        found_count += 1
    else:
        print(f"    [MISSING] {hex(addr)} -> {name}")

# Detailed tests for whichever sensors are online
print("\n[3] Functional Sensor Tests:")

# Test MLX90614 (0x5A)
if 0x5A in devices:
    try:
        raw = i2c.readfrom_mem(0x5A, 0x07, 3)
        temp_c = ((raw[0] | ((raw[1] & 0x7F) << 8)) * 0.02) - 273.15
        raw_amb = i2c.readfrom_mem(0x5A, 0x06, 3)
        temp_amb = ((raw_amb[0] | ((raw_amb[1] & 0x7F) << 8)) * 0.02) - 273.15
        print(f"    MLX90614: Object Temp = {temp_c:.2f} C | Ambient = {temp_amb:.2f} C [OK]")
    except Exception as e:
        print(f"    MLX90614 Read Error: {e}")
else:
    print("    MLX90614 (0x5A): Skipped (not detected on bus)")

# Test MAX30102 (0x57)
if 0x57 in devices:
    try:
        from max30102 import MAX30102
        max_sensor = MAX30102(i2c, addr=0x57)
        time.sleep_ms(100)
        red, ir = max_sensor._read_fifo()
        finger = "YES" if ir > 35000 else "NO"
        print(f"    MAX30102: Initialized! Sample -> Red={red}, IR={ir} (Finger={finger}) [OK]")
    except Exception as e:
        print(f"    MAX30102 Error: {e}")
else:
    print("    MAX30102 (0x57): Skipped (not detected on bus)")

# Test MPU6050 (0x68)
if 0x68 in devices:
    try:
        from mpu6050 import MPU6050
        mpu = MPU6050(i2c, addr=0x68, accel_range=2)
        ax, ay, az = mpu.accel()
        mag = mpu.accel_magnitude()
        print(f"    MPU6050: Accel = ({ax:.2f}, {ay:.2f}, {az:.2f}) g | Mag = {mag:.2f} g [OK]")
    except Exception as e:
        print(f"    MPU6050 Error: {e}")
else:
    print("    MPU6050 (0x68): Skipped (not detected on bus)")

# Test SSD1306 (0x3C)
if 0x3C in devices:
    try:
        from ssd1306 import SSD1306_I2C
        oled = SSD1306_I2C(128, 64, i2c, addr=0x3C)
        oled.fill(0)
        oled.rect(0, 0, 128, 64, 1)
        oled.text("DIAGNOSTIC TEST", 4, 10, 1)
        oled.text("OLED: ONLINE", 4, 25, 1)
        oled.text(f"Devs: {len(devices)}/4", 4, 40, 1)
        oled.show()
        print("    SSD1306: Screen test pattern drawn [OK]")
    except Exception as e:
        print(f"    SSD1306 Error: {e}")
else:
    print("    SSD1306 (0x3C): Skipped (not detected on bus)")

# Test SOS Button (GPIO 5) & Vibration Motor (GPIO 18)
print("\n[4] GPIO Peripherals:")
btn = Pin(PIN_SOS, Pin.IN, Pin.PULL_UP)
motor = Pin(PIN_MOTOR, Pin.OUT, value=0)
print(f"    SOS Button (GPIO {PIN_SOS}): Value = {btn.value()} ({'RELEASED' if btn.value() == 1 else 'PRESSED'})")
print(f"    Pulsing Vibration Motor on GPIO {PIN_MOTOR} for 150ms...")
motor.value(1)
time.sleep_ms(150)
motor.value(0)
print("    Vibration motor test complete.")

# Recommendations if bus is empty
if found_count == 0:
    print("\n" + "!" * 55)
    print("! NO I2C DEVICES DETECTED (Scan returned [])")
    print("! 1. Check breadboard center split (rows 30-31).")
    print("!    Bridge top & bottom power rails with jumper wires!")
    print("! 2. Check power to sensor: VCC -> 3.3V, GND -> GND.")
    print("! 3. Check SDA/SCL lines:")
    print(f"!    SDA -> ESP32 GPIO {PIN_SDA}")
    print(f"!    SCL -> ESP32 GPIO {PIN_SCL}")
    print("! 4. Try swapping SDA & SCL wires if reversed.")
    print("!" * 55)
elif found_count < 4:
    print(f"\n[INFO] {found_count} of 4 devices online. Connect remaining sensors to Row 40 (SCL) and Row 42 (SDA).")
else:
    print("\n[SUCCESS] ALL 4 I2C SENSORS ONLINE AND VERIFIED!")

print("=" * 55)
