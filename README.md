# IoMT Remote Patient Monitoring (RPM) Node

An ESP32-S3 powered Internet of Medical Things (IoMT) node for continuous, non-invasive vital signs monitoring, fall detection, and emergency alerting. Built with **C++ / PlatformIO** (Arduino framework).

## 🏥 Overview

Multi-sensor wearable patient monitoring node with an onboard OLED display, dual-bus I2C architecture, and structured JSON telemetry over serial for dashboards and alert checkers.

### Key Features
- **Heart Rate & SpO₂** — MAX30102 optical pulse oximeter (I2C Bus 0: `0x57`)
- **Body & Skin Temperature** — MLX90614 contactless IR thermometer (I2C Bus 0: `0x5A`)
- **Motion & Fall Detection** — MPU-6050 6-axis IMU (Dedicated I2C Bus 1: `0x68` / `0x69`)
- **OLED Display** — SSD1306 128×64, real-time vitals dashboard (I2C Bus 0: `0x3C`)
- **SOS Emergency Button** — GPIO 5, internal pull-up, active LOW
- **Dual Hardware I2C Buses** — Dedicated bus for MPU-6050 to prevent bus congestion
- **Telemetry Stream** — Formatted terminal card + compact single-line JSON stream at 115200 baud

---

## 📁 Project Files

| File | Purpose |
|:---|:---|
| `src/main.cpp` | Complete firmware — dual I2C buses, sensor drivers, fallback registers, OLED rendering, and JSON streaming |
| `platformio.ini` | PlatformIO environment config for ESP32-S3-DevKitC-1-N8 |
| `dashboard.html` | Browser-based live GUI connecting directly via WebSerial (no local server needed) |
| `ai_decision_engine.py` | Python serial listener that evaluates vitals against baseline alert thresholds |
| `wiring-guide.html` | Interactive hardware pinout and wiring reference |

---

## 🛠️ Hardware & Pin Configuration (ESP32-S3)

### Primary I2C Bus (`Wire` - Bus 0 at 50 kHz)
| Component | SDA Pin | SCL Pin | I2C Address | Notes |
|:---|:---|:---|:---|:---|
| **SSD1306 OLED** | GPIO 8 | GPIO 9 | `0x3C` | Real-time screen |
| **MAX30102 Pulse Ox** | GPIO 8 | GPIO 9 | `0x57` | Pulse & SpO2 |
| **MLX90614 IR Temp** | GPIO 8 | GPIO 9 | `0x5A` | 3.3V operation, SMBus compliant |

### Dedicated Secondary I2C Bus (`Wire1` - Bus 1 at 400 kHz)
| Component | SDA Pin | SCL Pin | I2C Address | Notes |
|:---|:---|:---|:---|:---|
| **MPU-6050 IMU** | GPIO 17 | GPIO 18 | `0x68` / `0x69` | Dedicated bus for fast motion & fall tracking |

### Peripherals
| Component | Pin | Configuration | Notes |
|:---|:---|:---|:---|
| **SOS Pushbutton** | GPIO 5 | `INPUT_PULLUP` | Connect other side to GND (Active LOW) |

> 💡 **Sensors & Power**: All sensor breakout boards operate on the 3.3V rail and include onboard I2C pull-up resistors.

---

> ⚕️ **Disclaimer**: This is an educational and engineering prototype. Vital sign calculations are approximations and are not intended to diagnose disease or prescribe medical treatments.

---

## 📡 Telemetry Format

Single-line JSON output over serial at `115200` baud:

```json
[JSON] {"uptime_s":45,"device_id":"RPM-NODE-01","risk":"NORMAL","sensors":{"oled":true,"mpu":true,"mlx":true,"max":true,"active":4},"vitals":{"heart_rate":72,"spo2":98,"finger_on":true,"temp_core":36.6,"temp_skin":33.6,"temp_amb":26.1,"steps":24,"motion_g":1.02},"alerts":{"sos":false,"fall":false},"ai_context":"All vital signs within expected baseline."}
```

### Alert Thresholds
| Condition | Threshold | System Action |
|:---|:---|:---|
| Low Heart Rate | BPM < 50 | Warning flag — notify caregiver |
| High Heart Rate | BPM > 120 | Warning flag — notify caregiver |
| Low SpO₂ | < 90% | Warning flag — notify caregiver |
| Elevated Temp | Core temp > 38.0°C | Warning flag — notify caregiver |
| Fall Detection | Acceleration > 2.5g | Emergency alert — check on patient |
| SOS Button | GPIO 5 pressed | Emergency alert — check on patient |

---

## 🚀 Building & Flashing

```powershell
# Compile the firmware
pio run

# Flash to ESP32-S3 over COM6
pio run -t upload --upload-port COM6

# View live telemetry in terminal
pio device monitor
```

### Live Monitoring Interfaces

1. **Terminal Stream**: Run `pio device monitor` to see real-time formatted vitals cards and raw JSON packets.
2. **Web Dashboard (`dashboard.html`)**: Open `dashboard.html` in Chrome or Edge, click **Connect**, and view live vitals and threshold alert status without installing additional tools.
3. **Threshold Alert Engine (`ai_decision_engine.py`)**: Run `python ai_decision_engine.py COM6` to inspect incoming packets and print status cards.

---

## 📋 Alert Behavior

When any alert condition is met:
1. **OLED Display** — Shows an emergency override screen (`EMERGENCY ALERT` / `SOS CALL` / `FALL ALERT`).
2. **Serial Stream** — Immediately flags `"risk": "EMERGENCY"` or `"WARNING"` with an alert description.
3. **Dashboard / Console** — Highlights the affected parameter and prompts the user to check on the patient.

