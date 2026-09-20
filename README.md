# IoMT Remote Patient Monitoring (RPM) Node — MicroPython

An ESP32-S3 powered Internet of Medical Things (IoMT) node for continuous, non-invasive remote patient monitoring, fall detection, and emergency alerting. Written in **MicroPython**.

## 🏥 Overview

Multi-sensor wrist-worn patient monitor with OLED display, haptic alerts, and structured JSON telemetry over serial.

### Key Features
- **Heart Rate & SpO₂** — MAX30102 optical pulse oximeter (I2C 0x57)
- **Skin Temperature** — MLX90614 contactless IR thermometer (I2C 0x5A)
- **Fall Detection** — MPU6050 6-axis IMU, acceleration magnitude threshold (I2C 0x68)
- **OLED Display** — SSD1306 128×64, real-time vitals dashboard (I2C 0x3C)
- **SOS Emergency Button** — GPIO 5, internal pull-up, active LOW
- **Haptic Alert** — Vibration motor via MOSFET on GPIO 18
- **JSON Telemetry** — Serial output at 115200 baud

---

## 📁 Project Files

| File | Purpose |
|:---|:---|
| `boot.py` | MicroPython boot config (disables debug, runs GC) |
| `main.py` | Main application — sensor loop, alerting, display, telemetry |
| `ssd1306.py` | SSD1306 OLED I2C driver (framebuf-based) |
| `mpu6050.py` | MPU6050 IMU driver (accelerometer + gyro) |
| `max30102.py` | MAX30102 pulse oximeter driver (BPM + SpO₂) |
| `mlx90614.py` | MLX90614 IR temperature driver |
| `test_sensors.py` | One-command hardware & I2C sensor diagnostic tool |
| `upload_to_esp32.py` | Automated PC-to-ESP32 flasher with auto-port detection |
| `wiring-guide.html` | Interactive hardware wiring reference |

---

## 🛠️ Hardware & Pin Configuration (ESP32-S3)

| Component | Interface / Pin | I2C Address |
|:---|:---|:---|
| **SSD1306 OLED** | I2C (SDA: GPIO 8, SCL: GPIO 9) | `0x3C` |
| **MPU6050 IMU** | I2C (SDA: GPIO 8, SCL: GPIO 9) | `0x68` |
| **MAX30102 Pulse Ox** | I2C (SDA: GPIO 8, SCL: GPIO 9) | `0x57` |
| **MLX90614 IR Temp** | I2C (SDA: GPIO 8, SCL: GPIO 9) | `0x5A` |
| **SOS Pushbutton** | GPIO 5 (INPUT_PULLUP) | — |
| **Vibration Motor** | GPIO 18 (OUTPUT via MOSFET) | — |
| **MAX30102 INT** | GPIO 2 (optional) | — |

> ⚠️ **MLX90614**: Must use the **3.3V version (BCC)**, not the 5V version (BAA).
> AD0 pin on MPU6050 must be tied to GND (sets address to 0x68).
> 💡 **I2C Signal Integrity**: Use a pair of **4.7kΩ pull-up resistors** (one SDA->3.3V, one SCL->3.3V) on breadboard builds. Internal MCU pull-ups (~45kΩ) are weak for 4-device shared buses.

---

> ⚕️ **Disclaimer**: This is an IoMT engineering prototype / academic demonstrator. Empirical vitals calculations are standard research approximations and are not intended for clinical or medical diagnostic use.

---

## 📡 Telemetry Format

JSON output over serial at `115200` baud, once per second:

```json
{
  "bpm": 76,
  "spo2": 98,
  "temp_c": 36.8,
  "fall": false,
  "sos": false,
  "alert": false,
  "uptime_s": 42
}
```

### Alert Thresholds
| Condition | Threshold |
|:---|:---|
| Low Heart Rate | BPM < 50 |
| High Heart Rate | BPM > 120 |
| Low SpO₂ (Hypoxemia) | < 90% |
| Fever | Core temp > 38.5°C |
| Fall Detection | Acceleration > 2.5g |

---

## 🚀 Getting Started

### Prerequisites
- ESP32-S3-DevKitC-1 board
- MicroPython firmware flashed ([download](https://micropython.org/download/ESP32_GENERIC_S3/))
- [mpremote](https://docs.micropython.org/en/latest/reference/mpremote.html) or [Thonny IDE](https://thonny.org/)

### 1. Flash MicroPython Firmware

```bash
# Download the latest ESP32-S3 MicroPython firmware (.bin)
# Then flash using esptool:
pip install esptool
esptool.py --chip esp32s3 --port COM6 erase_flash
esptool.py --chip esp32s3 --port COM6 write_flash -z 0x0 ESP32_GENERIC_S3-*.bin
```

### 2. Upload Project Files

```bash
# Using mpremote:
pip install mpremote
mpremote connect COM6 cp boot.py :boot.py
mpremote connect COM6 cp main.py :main.py
mpremote connect COM6 cp ssd1306.py :ssd1306.py
mpremote connect COM6 cp mpu6050.py :mpu6050.py
mpremote connect COM6 cp max30102.py :max30102.py
mpremote connect COM6 cp mlx90614.py :mlx90614.py
```

Or use **Thonny IDE**: Open each file → Save As → MicroPython device.

### 3. Monitor Output

```bash
mpremote connect COM6 repl
# Or any serial monitor at 115200 baud
```

---

## 📋 Clinical Alert Behavior

When any alert condition is active:
1. **OLED** — Top bar inverts to show `!! ALERT !!`
2. **Vibration Motor** — Pulses 200ms per loop cycle
3. **Serial** — `"alert": true` in JSON telemetry
4. **Fall Detection** — Auto-clears after 10 second cooldown

SOS button triggers immediate alert when pressed (active LOW, debounced).
