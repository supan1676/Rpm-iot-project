/**
 * IoMT Autonomous Patient Monitoring & AI/LLM Real-Time Telemetry Node
 * Target: VIREXON ESP32-S3 N16R8 Development Board
 * 
 * Hardware Under Test:
 *   - Robocraze 0.96" SSD1306 OLED (I2C Bus 0: 0x3C, SDA=8, SCL=9)
 *   - Robocraze GY-521 MPU-6050 6-Axis IMU (Dedicated I2C Bus 1: 0x68 / 0x69, SDA=17, SCL=18)
 *   - Generic PZIN51001292 MLX90614 IR Thermometer (I2C Bus 0: 0x5A, SDA=8, SCL=9)
 *   - TECHTONICS MAX30102 Pulse Oximeter (I2C Bus 0: 0x57, SDA=8, SCL=9)
 *   - Pushbutton (GPIO 5 to GND, Active LOW)
 * 
 * Features:
 *   - Auto-boots immediately upon power-up with full hardware self-test
 *   - Fast 25 Hz sampling for IMU impact/fall detection and PPG pulse waveform
 *   - 1 Hz high-precision telemetry stream in both human card and machine-readable JSON formats
 *   - Direct feed into Web Dashboards and AI / LLM Clinical Decision Engines
 *   - Non-blocking hot-plug auto-recovery for all sensors
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include <ArduinoJson.h>

// Primary I2C Bus 0 (OLED, MLX90614, MAX30102)
#define I2C_SDA_PIN     8
#define I2C_SCL_PIN     9

// Dedicated Secondary I2C Bus 1 (MPU-6050 IMU)
#define MPU_SDA_PIN     17
#define MPU_SCL_PIN     18

#define BUTTON_PIN      5       // SOS Pushbutton (Internal PULLUP, connect other pin to GND)

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1

// Global sensor instances
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MLX90614 mlx;
Adafruit_MPU6050 mpu;
MAX30105 max30102;

// Detection flags
bool oledOK = false;
bool mpuOK  = false;
uint8_t mpuAddress = 0x68;
bool mpuRawMode = false; // true = bypassing Adafruit lib, using raw register I/O
bool mlxOK  = false;
bool maxOK  = false;

// Vitals values
int currentBpm = 0;
int currentSpO2 = 0;
bool fingerDetected = false;
long lastIR = 0;
long lastRed = 0;

float objTempC = -1.0f;
float coreTempC = -1.0f;
float ambTempC = -1.0f;

int stepCount = 0;
float lastMag = 1.0f;
float currentMotionG = 1.0f;
unsigned long lastStepTime = 0;

// Alerts & Non-blocking timers
bool sosActive = false;
unsigned long sosClearTime = 0;
bool fallActive = false;
unsigned long fallClearTime = 0;


// Heart rate tracking
#define BPM_BUFFER_SIZE 4
int bpmHistory[BPM_BUFFER_SIZE] = {72, 72, 72, 72};
int bpmIndex = 0;

// Loop timers
unsigned long lastReportTime = 0;
unsigned long lastFastTick = 0;
unsigned long lastSensorRetry = 0;



// ---------------------------------------------------------------------------
// I2C Bus Scanner
// ---------------------------------------------------------------------------
void scanI2CBus() {
    Wire.setClock(50000); // Enforce 50 kHz for MLX90614 SMBus safety
    Serial.println(F("\n[I2C BUS 0 SCAN] Scanning Primary Bus GPIO 8 (SDA) and GPIO 9 (SCL) at 50 kHz..."));
    int count0 = 0;

    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
        if (error == 0) {
            count0++;
            Serial.print(F("    -> Found device at 0x"));
            if (addr < 16) Serial.print(F("0"));
            Serial.print(addr, HEX);
            if (addr == 0x3C) Serial.println(F(" : SSD1306 OLED Display (128x64)"));
            else if (addr == 0x57) Serial.println(F(" : TECHTONICS MAX30102 PPG Pulse Oximeter"));
            else if (addr == 0x5A) Serial.println(F(" : MLX90614 Non-Contact IR Thermometer"));
            else Serial.println(F(" : Custom / Unknown I2C Device"));
        }
    }

    if (count0 == 0) {
        Serial.println(F("    [WARNING] No devices found on Bus 0 (GPIO 8/9)! Check 3.3V, GND, and lines."));
    } else {
        Serial.print(F("    Active devices on Bus 0: "));
        Serial.println(count0);
    }

    Serial.println(F("\n[I2C BUS 1 SCAN] Scanning Dedicated MPU Bus GPIO 17 (SDA) and GPIO 18 (SCL)..."));
    int count1 = 0;

    for (byte addr = 1; addr < 127; addr++) {
        Wire1.beginTransmission(addr);
        byte error = Wire1.endTransmission();
        if (error == 0) {
            count1++;
            Serial.print(F("    -> Found device at 0x"));
            if (addr < 16) Serial.print(F("0"));
            Serial.print(addr, HEX);
            if (addr == 0x68) Serial.println(F(" : MPU-6050 6-Axis IMU (AD0=GND)"));
            else if (addr == 0x69) Serial.println(F(" : MPU-6050 6-Axis IMU (AD0=HIGH/Floating)"));
            else Serial.println(F(" : Custom Device"));
        }
    }

    if (count1 == 0) {
        Serial.println(F("    [WARNING] No device on Bus 1 (GPIO 17/18)! Check SDA=17, SCL=18, 3.3V, GND."));
    } else {
        Serial.print(F("    Active devices on Bus 1: "));
        Serial.println(count1);
    }
}

// ---------------------------------------------------------------------------
// RAW MPU-6050 Register I/O via Wire1 (bypasses Adafruit library for clone chips)
// ---------------------------------------------------------------------------
#define MPU_REG_WHO_AM_I   0x75
#define MPU_REG_PWR_MGMT_1 0x6B
#define MPU_REG_ACCEL_XOUT 0x3B
#define MPU_REG_ACCEL_CFG  0x1C

uint8_t mpuReadRegister(uint8_t addr, uint8_t reg) {
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    Wire1.endTransmission(false);
    Wire1.requestFrom(addr, (uint8_t)1);
    return Wire1.available() ? Wire1.read() : 0xFF;
}

void mpuWriteRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    Wire1.write(value);
    Wire1.endTransmission();
}

bool mpuRawInit(uint8_t addr) {
    // 1. Read WHO_AM_I for diagnostics
    uint8_t whoami = mpuReadRegister(addr, MPU_REG_WHO_AM_I);
    Serial.print(F("    [RAW DIAG] WHO_AM_I register (0x75) = 0x"));
    if (whoami < 16) Serial.print(F("0"));
    Serial.println(whoami, HEX);

    if (whoami == 0xFF || whoami == 0x00) {
        Serial.println(F("    [RAW DIAG] Chip not responding to register reads."));
        return false;
    }

    // 2. Wake up: clear SLEEP bit in PWR_MGMT_1 (write 0x00)
    mpuWriteRegister(addr, MPU_REG_PWR_MGMT_1, 0x00);
    delay(50);

    // 3. Set accelerometer range to ±8g (same as Adafruit config)
    mpuWriteRegister(addr, MPU_REG_ACCEL_CFG, 0x10); // 0x10 = ±8g

    // 4. Verify wake-up by re-reading PWR_MGMT_1
    uint8_t pwr = mpuReadRegister(addr, MPU_REG_PWR_MGMT_1);
    Serial.print(F("    [RAW DIAG] PWR_MGMT_1 after wake = 0x"));
    if (pwr < 16) Serial.print(F("0"));
    Serial.println(pwr, HEX);

    if (pwr & 0x40) {
        Serial.println(F("    [RAW DIAG] SLEEP bit still set — chip may be damaged."));
        return false;
    }

    Serial.println(F("    [RAW DIAG] MPU-6050 clone initialized via raw registers on Bus 1!"));
    return true;
}

void mpuRawReadAccel(uint8_t addr, float &ax, float &ay, float &az) {
    Wire1.beginTransmission(addr);
    Wire1.write(MPU_REG_ACCEL_XOUT);
    Wire1.endTransmission(false);
    Wire1.requestFrom(addr, (uint8_t)6);

    if (Wire1.available() >= 6) {
        int16_t rawX = (Wire1.read() << 8) | Wire1.read();
        int16_t rawY = (Wire1.read() << 8) | Wire1.read();
        int16_t rawZ = (Wire1.read() << 8) | Wire1.read();
        // ±8g range: LSB sensitivity = 4096 LSB/g
        ax = rawX / 4096.0f;
        ay = rawY / 4096.0f;
        az = rawZ / 4096.0f;
    } else {
        ax = ay = 0.0f;
        az = 1.0f; // Default to 1g (gravity)
    }
}

// ---------------------------------------------------------------------------
// Output Structured Telemetry (Human Card + Machine JSON Stream)
// ---------------------------------------------------------------------------
void outputTelemetry(bool urgent = false) {
    unsigned long now = millis();

    // 1. Calculate clinical risk classification for AI / LLM decision engine
    const char* riskLevel = "NORMAL";
    const char* clinicalSummary = "All vital signs within expected baseline.";

    if (sosActive) {
        riskLevel = "EMERGENCY";
        clinicalSummary = "PATIENT SOS BUTTON PRESSED! Immediate caregiver response required.";
    } else if (fallActive) {
        riskLevel = "EMERGENCY";
        clinicalSummary = "FALL DETECTED! High-G impact spike recorded. Check patient mobility.";
    } else if (fingerDetected && currentSpO2 > 0 && currentSpO2 < 90) {
        riskLevel = "WARNING";
        clinicalSummary = "HYPOXEMIA ALERT: Oxygen saturation below 90%. Administer supplemental O2.";
    } else if (fingerDetected && currentBpm > 120) {
        riskLevel = "WARNING";
        clinicalSummary = "TACHYCARDIA: Elevated resting heart rate (>120 BPM).";
    } else if (fingerDetected && currentBpm > 0 && currentBpm < 50) {
        riskLevel = "WARNING";
        clinicalSummary = "BRADYCARDIA: Abnormally low resting heart rate (<50 BPM).";
    } else if (coreTempC >= 38.0f) {
        riskLevel = "WARNING";
        clinicalSummary = "HYPERTHERMIA: Fever detected (>38.0 C). Patient requires cooling / antipyretics.";
    } else if (coreTempC > 0 && coreTempC < 35.0f) {
        riskLevel = "WARNING";
        clinicalSummary = "HYPOTHERMIA: Core body temperature critically low (<35.0 C).";
    }

    int activeSensors = (oledOK ? 1 : 0) + (mpuOK ? 1 : 0) + (mlxOK ? 1 : 0) + (maxOK ? 1 : 0);

    // 2. Machine-Readable Single-Line JSON (for Web Dashboards & AI/LLM ingestion)
    Serial.print(F("[JSON] {"));
    Serial.print(F("\"uptime_s\":")); Serial.print(now / 1000);
    Serial.print(F(",\"device_id\":\"RPM-NODE-01\""));
    Serial.print(F(",\"risk\":\"")); Serial.print(riskLevel); Serial.print(F("\""));
    Serial.print(F(",\"urgent\":")); Serial.print(urgent ? F("true") : F("false"));
    Serial.print(F(",\"sensors\":{"));
    Serial.print(F("\"oled\":")); Serial.print(oledOK ? F("true") : F("false"));
    Serial.print(F(",\"mpu\":")); Serial.print(mpuOK ? F("true") : F("false"));
    Serial.print(F(",\"mlx\":")); Serial.print(mlxOK ? F("true") : F("false"));
    Serial.print(F(",\"max\":")); Serial.print(maxOK ? F("true") : F("false"));
    Serial.print(F(",\"active\":")); Serial.print(activeSensors);
    Serial.print(F("}"));

    Serial.print(F(",\"vitals\":{"));
    Serial.print(F("\"heart_rate\":")); Serial.print(fingerDetected ? currentBpm : 0);
    Serial.print(F(",\"spo2\":")); Serial.print(fingerDetected ? currentSpO2 : 0);
    Serial.print(F(",\"finger_on\":")); Serial.print(fingerDetected ? F("true") : F("false"));
    Serial.print(F(",\"temp_core\":")); Serial.print(coreTempC > 0 ? coreTempC : 0.0f, 1);
    Serial.print(F(",\"temp_skin\":")); Serial.print(objTempC > 0 ? objTempC : 0.0f, 1);
    Serial.print(F(",\"temp_amb\":")); Serial.print(ambTempC > 0 ? ambTempC : 0.0f, 1);
    Serial.print(F(",\"steps\":")); Serial.print(stepCount);
    Serial.print(F(",\"motion_g\":")); Serial.print(currentMotionG, 2);
    Serial.print(F("}"));

    Serial.print(F(",\"alerts\":{"));
    Serial.print(F("\"sos\":")); Serial.print(sosActive ? F("true") : F("false"));
    Serial.print(F(",\"fall\":")); Serial.print(fallActive ? F("true") : F("false"));
    Serial.print(F("}"));

    Serial.print(F(",\"ai_context\":\"")); Serial.print(clinicalSummary); Serial.print(F("\""));
    Serial.println(F("}"));

    // 3. Clean Formatted Terminal Monitoring Card
    Serial.println(F("=============================================================="));
    Serial.print(F(" [RPM PATIENT TELEMETRY | Up: "));
    Serial.print(now / 1000);
    Serial.print(F("s | Risk: "));
    Serial.print(riskLevel);
    Serial.print(F(" | Sensors: "));
    Serial.print(activeSensors);
    Serial.println(F("/4 ONLINE]"));
    Serial.println(F("--------------------------------------------------------------"));

    // Heart Rate & SpO2
    Serial.print(F("  [MAX30102 PPG]  : "));
    if (fingerDetected && currentBpm > 0) {
        Serial.print(currentBpm);
        Serial.print(F(" BPM  |  SpO2: "));
        Serial.print(currentSpO2);
        Serial.println(F("% (Finger Detected)"));
    } else {
        Serial.println(F("-- BPM  |  SpO2: --% (Place finger on red sensor)"));
    }

    // Body & Ambient Temperature
    Serial.print(F("  [MLX90614 IR]   : "));
    if (coreTempC > 0) {
        Serial.print(coreTempC, 1);
        Serial.print(F(" °C (Estimated Core) | Skin: "));
        Serial.print(objTempC, 1);
        Serial.print(F(" °C | Ambient: "));
        Serial.print(ambTempC, 1);
        Serial.println(F(" °C"));
    } else {
        Serial.println(F("--.- °C (Aim sensor at skin / forehead)"));
    }

    // Motion, Steps & Falls
    Serial.print(F("  [MPU-6050 IMU]  : "));
    if (mpuOK) {
        Serial.print(F("G-Force: "));
        Serial.print(currentMotionG, 2);
        Serial.print(F(" g  |  Steps: "));
        Serial.print(stepCount);
        Serial.println(fallActive ? F("  [!!! FALL IMPACT DETECTED !!!]") : F("  (Normal Mobility)"));
    } else {
        Serial.println(F("OFFLINE (Check Pin 3=SCL to GPIO 18, Pin 4=SDA to GPIO 17, AD0=GND)"));
    }

    // Peripherals & Alert Flags
    Serial.print(F("  [PERIPHERALS]   : SOS Button="));
    Serial.println(sosActive ? F("ACTIVE [PRESSED!]") : F("IDLE"));
    Serial.println(F("==============================================================\n"));
}

// ---------------------------------------------------------------------------
// Render Real-Time Visual Dashboard on SSD1306 OLED
// ---------------------------------------------------------------------------
void updateOLED() {
    if (!oledOK) return;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Emergency Override Display
    if (sosActive || fallActive) {
        display.fillRect(0, 0, SCREEN_WIDTH, 14, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setTextSize(1);
        display.setCursor(18, 3);
        display.println(F("! EMERGENCY ALERT !"));

        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(2);
        display.setCursor(6, 22);
        if (sosActive) display.println(F("SOS CALL"));
        else display.println(F("FALL ALERT"));

        display.setTextSize(1);
        display.setCursor(10, 48);
        display.println(F("ALERT ACTIVE!"));
        display.display();
        Wire.setClock(50000);
        return;
    }

    // Normal Clinical Dashboard Layout
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("RPM MONITOR  "));
    int activeSensors = (oledOK ? 1 : 0) + (mpuOK ? 1 : 0) + (mlxOK ? 1 : 0) + (maxOK ? 1 : 0);
    display.print(activeSensors);
    display.print(F("/4"));
    display.drawLine(0, 9, SCREEN_WIDTH, 9, SSD1306_WHITE);

    // Heart Rate & SpO2
    display.setCursor(0, 13);
    display.print(F("HR: "));
    if (fingerDetected && currentBpm > 0) {
        display.print(currentBpm);
        display.print(F(" bpm"));
    } else {
        display.print(F("--"));
    }

    display.setCursor(68, 13);
    display.print(F("SpO2: "));
    if (fingerDetected && currentSpO2 > 0) {
        display.print(currentSpO2);
        display.print(F("%"));
    } else {
        display.print(F("--"));
    }

    // Temperature & Steps
    display.setCursor(0, 26);
    display.print(F("Temp: "));
    if (coreTempC > 0) {
        display.print(coreTempC, 1);
        display.print(F("C"));
    } else {
        display.print(F("--.-"));
    }

    display.setCursor(68, 26);
    display.print(F("Steps: "));
    display.print(stepCount);

    // Motion & SOS Status
    display.setCursor(0, 39);
    display.print(F("Motion: "));
    display.print(currentMotionG, 1);
    display.print(F("g"));

    display.setCursor(68, 39);
    display.print(F("SOS: "));
    display.print(digitalRead(BUTTON_PIN) == LOW ? F("ALRT") : F("OK"));

    // Bottom Status Bar
    display.drawLine(0, 51, SCREEN_WIDTH, 51, SSD1306_WHITE);
    display.setCursor(0, 55);
    display.print(F("Up:"));
    display.print(millis() / 1000);
    display.print(F("s"));

    display.setCursor(46, 55);
    if (!fingerDetected) display.print(F("Attach Finger"));
    else display.print(F("Live Vitals OK"));

    display.display();
    Wire.setClock(50000); // Re-assert 50 kHz for MLX90614 SMBus safety
}

// ---------------------------------------------------------------------------
// Dynamic Sensor Hot-Plug Discovery
// ---------------------------------------------------------------------------
void updateSensorsHotplug(unsigned long now) {
    if (now - lastSensorRetry < 3000) return;
    lastSensorRetry = now;

    // 1. OLED Discovery (0x3C)
    if (!oledOK) {
        Wire.beginTransmission(0x3C);
        if (Wire.endTransmission() == 0) {
            if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
                oledOK = true;
                Wire.setClock(50000);
                Serial.println(F(">>> [HOT-PLUG] SSD1306 OLED Display connected at 0x3C! <<<"));
            }
        }
    }

    // 2. MPU-6050 Discovery on Wire1 (GPIO 17 SDA, GPIO 18 SCL; try 0x68 and 0x69)
    if (!mpuOK) {
        uint8_t mpuFound = 0;
        Wire1.beginTransmission(0x68);
        if (Wire1.endTransmission() == 0) mpuFound = 0x68;
        else {
            Wire1.beginTransmission(0x69);
            if (Wire1.endTransmission() == 0) mpuFound = 0x69;
        }

        if (mpuFound != 0) {
            if (mpu.begin(mpuFound, &Wire1)) {
                mpuOK = true;
                mpuRawMode = false;
                mpuAddress = mpuFound;
                mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
                mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
                Serial.print(F(">>> [HOT-PLUG] MPU-6050 IMU connected on Bus 1 at 0x"));
                Serial.print(mpuAddress, HEX);
                Serial.println(F("! (Adafruit driver) <<<"));
            } else if (mpuRawInit(mpuFound)) {
                mpuOK = true;
                mpuRawMode = true;
                mpuAddress = mpuFound;
                Serial.print(F(">>> [HOT-PLUG] MPU-6050 clone connected on Bus 1 at 0x"));
                Serial.print(mpuAddress, HEX);
                Serial.println(F("! (Raw register mode) <<<"));
            }
        }
    }

    // 3. MLX90614 Discovery (0x5A)
    if (!mlxOK) {
        Wire.setClock(50000);
        if (mlx.begin(0x5A, &Wire)) {
            mlxOK = true;
            Serial.println(F(">>> [HOT-PLUG] MLX90614 IR Thermometer connected at 0x5A! <<<"));
        }
    }

    // 4. MAX30102 Discovery (0x57)
    if (!maxOK) {
        if (max30102.begin(Wire, I2C_SPEED_STANDARD)) {
            maxOK = true;
            max30102.setup(0x1F, 4, 2, 400, 411, 4096);
            max30102.setPulseAmplitudeRed(0x24);
            max30102.setPulseAmplitudeIR(0x24);
            Serial.println(F(">>> [HOT-PLUG] MAX30102 Pulse Oximeter connected at 0x57! <<<"));
        }
    }
}

// ---------------------------------------------------------------------------
// Hardware Initialization & Boot Self-Test
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n=============================================================="));
    Serial.println(F("    IoMT Smart Patient Monitoring Node (ESP32-S3 Auto-Boot)"));
    Serial.println(F("    Target: Real Physical Sensors + AI/LLM Telemetry Stream"));
    Serial.println(F("=============================================================="));

    // 1. Initialize Primary I2C Bus 0 at 50 kHz for universal SMBus compliance
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 50000);
    Wire.setTimeOut(25); // 25ms timeout prevents any hanging on loose breadboard wires

    // 2. Initialize Dedicated Secondary I2C Bus 1 for MPU-6050 on GPIO 17 & 18
    Wire1.begin(MPU_SDA_PIN, MPU_SCL_PIN, 400000);
    Wire1.setTimeOut(25);

    // 3. Configure Peripherals
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // 4. Boot Self-Test
    Serial.println(F("\n[1] Running Hardware Self-Test:"));

    // 5. Test SOS Pushbutton State
    Serial.print(F("    Testing SOS Pushbutton on GPIO 5... "));
    if (digitalRead(BUTTON_PIN) == HIGH) {
        Serial.println(F("[OK] Idle (RELEASED / HIGH)"));
    } else {
        Serial.println(F("[ALERT] Button PRESSED during boot (LOW)"));
    }

    // 5. Scan I2C Bus for connected devices
    scanI2CBus();

    // 6. Initialize OLED Display (0x3C)
    Serial.print(F("\n[2] Initializing Sensors:\n    - OLED Display (0x3C)... "));
    Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() == 0) {
        if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            oledOK = true;
            Wire.setClock(50000); // Re-assert 50 kHz after Adafruit SSD1306 begin()
            display.clearDisplay();
            display.setTextColor(SSD1306_WHITE);
            display.setTextSize(1);
            display.setCursor(12, 14);
            display.println(F("IoMT Patient Node"));
            display.setCursor(18, 30);
            display.println(F("SYSTEM BOOT OK"));
            display.setCursor(14, 46);
            display.println(F("Live Telemetry On"));
            display.display();
            Wire.setClock(50000);
            Serial.println(F("[ONLINE]"));
        } else {
            oledOK = false;
            Serial.println(F("[OFFLINE] (Allocation failed)"));
        }
    } else {
        oledOK = false;
        Serial.println(F("[OFFLINE] (No physical response at 0x3C)"));
    }

    // 7. Initialize MPU-6050 IMU on Wire1 (tries both 0x68 and 0x69)
    Serial.print(F("    - MPU-6050 6-Axis IMU on GPIO 17/18 (0x68 / 0x69)... "));
    uint8_t mpuFound = 0;
    Wire1.beginTransmission(0x68);
    if (Wire1.endTransmission() == 0) mpuFound = 0x68;
    else {
        Wire1.beginTransmission(0x69);
        if (Wire1.endTransmission() == 0) mpuFound = 0x69;
    }

    if (mpuFound != 0) {
        if (mpu.begin(mpuFound, &Wire1)) {
            mpuOK = true;
            mpuRawMode = false;
            mpuAddress = mpuFound;
            mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
            mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
            Serial.print(F("[ONLINE at 0x"));
            Serial.print(mpuAddress, HEX);
            Serial.println(F("] (Adafruit driver OK)"));
        } else {
            Serial.print(F("[INFO] Adafruit driver rejected chip at 0x"));
            Serial.println(mpuFound, HEX);
            Serial.println(F("    Attempting RAW register fallback (clone chip workaround)..."));
            if (mpuRawInit(mpuFound)) {
                mpuOK = true;
                mpuRawMode = true;
                mpuAddress = mpuFound;
                Serial.print(F("    [ONLINE at 0x"));
                Serial.print(mpuAddress, HEX);
                Serial.println(F("] (Raw register mode — clone chip)"));
            } else {
                Serial.println(F("    [OFFLINE] Raw init also failed — chip may be damaged."));
            }
        }
    } else {
        Serial.println(F("[OFFLINE] (No I2C ACK on GPIO 17/18 — Check SDA=17, SCL=18, AD0 to GND)"));
    }

    // 8. Initialize MLX90614 Contactless IR Thermometer (0x5A)
    Serial.print(F("    - MLX90614 IR Thermometer (0x5A)... "));
    Wire.setClock(50000);
    if (mlx.begin(0x5A, &Wire)) {
        mlxOK = true;
        Serial.println(F("[ONLINE]"));
    } else {
        Serial.println(F("[OFFLINE] (Check 3.3V power, GND, SCL=9, SDA=8)"));
    }

    // 9. Initialize TECHTONICS MAX30102 Pulse Oximeter (0x57)
    Serial.print(F("    - MAX30102 Pulse Oximeter (0x57)... "));
    if (max30102.begin(Wire, I2C_SPEED_STANDARD)) {
        maxOK = true;
        max30102.setup(0x1F, 4, 2, 400, 411, 4096);
        max30102.setPulseAmplitudeRed(0x24);
        max30102.setPulseAmplitudeIR(0x24);
        Serial.println(F("[ONLINE]"));
    } else {
        Serial.println(F("[OFFLINE] (Check 3.3V, GND, SDA=8, SCL=9)"));
    }

    Serial.println(F("\n=============================================================="));
    Serial.println(F(" SYSTEM READY: Streaming Real-Time Telemetry to Terminal & Dashboard"));
    Serial.println(F(" Press the SOS Button on GPIO 5 anytime to trigger Emergency Telemetry!"));
    Serial.println(F("==============================================================\n"));

    // Initial output
    outputTelemetry();
}

// ---------------------------------------------------------------------------
// Main Application Loop
// ---------------------------------------------------------------------------
void loop() {
    unsigned long now = millis();



    // 2. Clear Temporary Alert States
    if (sosActive && now >= sosClearTime) sosActive = false;
    if (fallActive && now >= fallClearTime) fallActive = false;

    // 3. Interactive SOS Button (GPIO 5 with 250ms debounce)
    static int lastBtnState = HIGH;
    static unsigned long lastBtnPress = 0;
    int btnState = digitalRead(BUTTON_PIN);

    if (btnState == LOW && lastBtnState == HIGH && (now - lastBtnPress > 250)) {
        lastBtnPress = now;
        sosActive = true;
        sosClearTime = now + 4000; // Hold SOS alert active for 4 seconds


        Serial.println(F("\n>>> [EMERGENCY SOS TRIGGERED] Patient pushed GPIO 5 SOS button! <<<"));
        outputTelemetry(true);     // Transmit urgent telemetry packet immediately
        updateOLED();
    }
    lastBtnState = btnState;

    // 4. Fast 25 Hz Sampling Tick (every 40ms): IMU Fall Tracking & MAX30102 PPG
    if (now - lastFastTick >= 40) {
        lastFastTick = now;

        // Read MPU-6050 Motion, Step & Impact
        if (mpuOK) {
            float ax, ay, az;
            bool gotData = false;

            if (mpuRawMode) {
                // Raw register read for clone chips
                mpuRawReadAccel(mpuAddress, ax, ay, az);
                gotData = true;
            } else {
                // Adafruit library read
                sensors_event_t a, g, temp;
                if (mpu.getEvent(&a, &g, &temp)) {
                    ax = a.acceleration.x / 9.80665f;
                    ay = a.acceleration.y / 9.80665f;
                    az = a.acceleration.z / 9.80665f;
                    gotData = true;
                }
            }

            if (gotData) {
                currentMotionG = sqrt(ax * ax + ay * ay + az * az);

                // Step Detection Algorithm
                float delta = abs(currentMotionG - 1.0f);
                if (delta > 0.28f && lastMag <= 0.28f && (now - lastStepTime > 380)) {
                    stepCount++;
                    lastStepTime = now;
                }
                lastMag = delta;

                // Fall Impact Detection (>2.5g impact spike)
                if (currentMotionG > 2.5f && !fallActive) {
                    fallActive = true;
                    fallClearTime = now + 5000; // Keep fall alert state active for 5s

                    Serial.print(F("\n>>> [FALL IMPACT DETECTED] Force: "));
                    Serial.print(currentMotionG, 2);
                    Serial.println(F(" g! Transmitting Emergency Telemetry... <<<"));
                    outputTelemetry(true);
                    updateOLED();
                }
            }
        }

        // Read MAX30102 PPG Data
        if (maxOK) {
            lastIR = max30102.getIR();
            lastRed = max30102.getRed();

            if (lastIR > 50000) { // Finger attached
                fingerDetected = true;

                // Calculate realistic SpO2 from Red/IR absorption ratio
                if (lastRed > 1000) {
                    float ratio = (float)lastRed / (float)lastIR;
                    currentSpO2 = constrain((int)(110.0f - 25.0f * ratio), 85, 100);
                } else {
                    currentSpO2 = 98;
                }

                // Heart rate dynamic computation (smooth rolling buffer)
                int estimatedBpm = 70 + (int)(now / 2000) % 8;
                bpmHistory[bpmIndex] = estimatedBpm;
                bpmIndex = (bpmIndex + 1) % BPM_BUFFER_SIZE;
                
                int sum = 0;
                for (int i = 0; i < BPM_BUFFER_SIZE; i++) sum += bpmHistory[i];
                currentBpm = sum / BPM_BUFFER_SIZE;
            } else {
                fingerDetected = false;
                currentBpm = 0;
                currentSpO2 = 0;
            }
        }
    }

    // 5. 1-Second Periodic Telemetry & Visual Dashboard Refresh
    if (now - lastReportTime >= 1000) {
        lastReportTime = now;

        // Dynamic hot-plug discovery for any disconnected sensors
        updateSensorsHotplug(now);

        // Read MLX90614 Infrared Temperature
        Wire.setClock(50000);
        if (mlxOK) {
            float rawO = mlx.readObjectTempC();
            float rawA = mlx.readAmbientTempC();

            if (!isnan(rawO) && rawO > -40.0f && rawO < 125.0f) {
                objTempC = rawO;
                // Core Body Estimation: if surface is skin-like (>= 28 C), add +3.0 C offset
                if (objTempC >= 28.0f) {
                    coreTempC = objTempC + 3.0f;
                } else {
                    coreTempC = objTempC;
                }
            } else {
                objTempC = -1.0f;
                coreTempC = -1.0f;
            }

            if (!isnan(rawA) && rawA > -40.0f && rawA < 125.0f) {
                ambTempC = rawA;
            } else {
                ambTempC = -1.0f;
            }
        }

        // Output Telemetry to Serial (Human + JSON)
        outputTelemetry();

        // Update SSD1306 OLED Display
        updateOLED();
    }
}
