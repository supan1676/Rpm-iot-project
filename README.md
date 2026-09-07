# IoMT Remote Patient Monitoring (RPM) Node

An ESP32-S3 powered Internet of Medical Things (IoMT) node designed for continuous, non-invasive remote patient monitoring, fall detection, and emergency alerting.

## 🏥 Overview

This project implements a multi-sensor bedside patient monitor with local visual feedback, auditory alarms, and structured JSON telemetry for IoT cloud / gateway ingestion.

### Key Features
- **Vital Signs Monitoring**:
  - Heart Rate (BPM) & Blood Oxygen Saturation ($SpO_2$) simulation / sensing
  - Ambient & Body Temperature and Relative Humidity via **DHT22**
- **Safety & Activity Tracking**:
  - **Fall Detection**: Real-time 3-axis accelerometer vector magnitude calculation using **MPU6050**
  - **Bed Presence / Mobility**: **PIR** motion sensing and **HC-SR04** ultrasonic distance measurement
- **Alerting & Local Display**:
  - **OLED SSD1306** (128x64 I2C): Displays real-time vitals and high-priority clinical warnings
  - **SOS Emergency Pushbutton**: Immediate manual trigger for patient distress calls
  - **Pulsing Buzzer**: Auditory alert system triggered by abnormal vitals, falls, or SOS
- **Structured Telemetry**:
  - Serial JSON stream emitting vitals, sensor flags, and alert statuses for downstream processing

---

## 🛠️ Hardware & Pin Configuration (ESP32-S3)

| Component | Interface / Pin | Description |
| :--- | :--- | :--- |
| **SSD1306 OLED** | I2C (SDA: GPIO 8, SCL: GPIO 9) | 128x64 display (Address: `0x3C`) |
| **MPU6050 IMU** | I2C (SDA: GPIO 8, SCL: GPIO 9) | 6-DOF IMU (Address: `0x68`) |
| **DHT22 Sensor** | GPIO 5 | Temperature & Humidity |
| **HR Potentiometer** | GPIO 4 (ADC) | Heart Rate (45 - 160 BPM) |
| **SpO2 Potentiometer** | GPIO 6 (ADC) | Oxygen Saturation (85% - 100%) |
| **PIR Motion Sensor** | GPIO 7 | Bed mobility / movement detection |
| **SOS Pushbutton** | GPIO 10 | Emergency button (Input Pull-up, Active LOW) |
| **HC-SR04 Ultrasonic** | TRIG: GPIO 15, ECHO: GPIO 16 | Bed proximity / distance measurement |
| **Piezo Buzzer** | GPIO 18 | Pulsed clinical alarm |

---

## 📡 Telemetry Format

The node outputs newline-delimited JSON over serial at `115200` baud:

```json
{
  "bpm": 76,
  "spo2": 98,
  "temp_c": 36.8,
  "humidity_pct": 55.2,
  "motion": false,
  "bed_dist_cm": 45.2,
  "fall": false,
  "sos": false,
  "alert": false,
  "uptime_s": 42
}
```

---

## 🚀 Getting Started

### Prerequisites
- [PlatformIO](https://platformio.org/) installed in VS Code or CLI
- (Optional) [Wokwi](https://wokwi.com/) for online simulation

### Build & Flash via PlatformIO
1. Clone this repository:
   ```bash
   git clone https://github.com/supan1676/Rpm-iot-project.git
   cd Rpm-iot-project
   ```
2. Build the project:
   ```bash
   pio run
   ```
3. Upload to ESP32-S3:
   ```bash
   pio run -t upload
   ```
4. Open the Serial Monitor:
   ```bash
   pio device monitor -b 115200
   ```

### Simulation via Wokwi
This project includes `wokwi.toml` and `diagram.json` for simulation:
1. Open the Wokwi extension in VS Code.
2. Press `F1` and choose **Wokwi: Start Simulator**.
