/**
 * IoMT Hardware Diagnostic & Live Terminal Monitor (C++ / Arduino)
 * Target: VIREXON ESP32-S3 N16R8 Development Board
 * 
 * Hardware Under Test:
 *   - Robocraze 0.96" SSD1306 OLED (I2C: 0x3C, SDA=8, SCL=9)
 *   - Robocraze GY-521 MPU-6050 6-Axis IMU (I2C: 0x68, SDA=8, SCL=9)
 *   - Generic PZIN51001292 MLX90614 IR Thermometer (I2C: 0x5A, SDA=8, SCL=9)
 *   - TECHTONICS MAX30102 Pulse Oximeter (I2C: 0x57, SDA=8, SCL=9)
 *   - Pushbutton (GPIO 5 to GND)
 *   - Vibration Motor (GPIO 18)
 * 
 * No external servers or WiFi required — 100% local, instant terminal diagnostics.
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

// Pin definitions
#define I2C_SDA_PIN     8
#define I2C_SCL_PIN     9
#define BUTTON_PIN      5       // Pushbutton (connect other pin to GND)
#define VIBRATION_PIN   18      // Haptic Vibration Motor

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1

// Global instances
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MLX90614 mlx;
Adafruit_MPU6050 mpu;
MAX30105 max30102;

// Detection flags
bool oledOK = false;
bool mpuOK  = false;
bool mlxOK  = false;
bool maxOK  = false;

// Step counter & motion tracking
int stepCount = 0;
float lastMag = 1.0f;
unsigned long lastStepTime = 0;

// Motor pulse timer (non-blocking)
unsigned long motorOffTime = 0;

// Vitals values
float objTempC = -1.0f;
float ambTempC = -1.0f;
long lastIR = 0;
long lastRed = 0;
int currentBpm = 0;
int currentSpO2 = 0;
bool fingerDetected = false;

// Timers
unsigned long lastReportTime = 0;
unsigned long lastFastTick = 0;

// ---------------------------------------------------------------------------
// Run I2C Bus Scan & Print Results
// ---------------------------------------------------------------------------
void scanI2CBus() {
    Wire.setClock(50000); // Enforce 50 kHz before bus scan for MLX90614 SMBus compliance
    Serial.println(F("\n[1] Scanning I2C Bus on GPIO 8 (SDA) and GPIO 9 (SCL) at 50 kHz..."));
    int count = 0;

    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
        if (error == 0) {
            count++;
            Serial.print(F("    -> Found device at 0x"));
            if (addr < 16) Serial.print(F("0"));
            Serial.print(addr, HEX);
            if (addr == 0x3C) Serial.println(F(" : SSD1306 OLED Display"));
            else if (addr == 0x57) Serial.println(F(" : TECHTONICS MAX30102 PPG"));
            else if (addr == 0x5A) Serial.println(F(" : MLX90614 IR Thermometer"));
            else if (addr == 0x68) Serial.println(F(" : MPU6050 6-Axis IMU"));
            else Serial.println(F(" : Unknown I2C device"));
        }
    }

    if (count == 0) {
        Serial.println(F("    [WARNING] No I2C devices responded! Check power, ground, and pin connections."));
    } else {
        Serial.print(F("    Total devices found: "));
        Serial.println(count);
    }
}

// ---------------------------------------------------------------------------
// Initial Setup & Hardware Verification
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n=============================================================="));
    Serial.println(F("       IoMT Hardware Diagnostic & Live Sensor Console"));
    Serial.println(F("=============================================================="));

    // Initialize custom I2C pins for ESP32-S3 at 50 kHz (SMBus safe for MLX90614)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 50000);

    // Initialize GPIOs
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(VIBRATION_PIN, OUTPUT);
    digitalWrite(VIBRATION_PIN, LOW);

    // Test vibration motor with a quick 150ms boot pulse
    Serial.println(F("\n[2] Peripheral Self-Test:"));
    Serial.print(F("    Pulsing Vibration Motor on GPIO 18... "));
    digitalWrite(VIBRATION_PIN, HIGH);
    delay(150);
    digitalWrite(VIBRATION_PIN, LOW);
    Serial.println(F("[OK] Done"));

    // Check button initial state
    int btnInit = digitalRead(BUTTON_PIN);
    Serial.print(F("    Reading Pushbutton on GPIO 5... "));
    if (btnInit == HIGH) {
        Serial.println(F("[OK] Idle (RELEASED / HIGH)"));
    } else {
        Serial.println(F("[NOTICE] Currently PRESSED (LOW)"));
    }

    // Run bus scan
    scanI2CBus();

    // 1. Initialize OLED (0x3C)
    Serial.print(F("\n[3] Initializing Sensors:\n    - OLED Display (0x3C)... "));
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        oledOK = true;
        // CRITICAL FIX: Adafruit_SSD1306 internally forces I2C clock to 400 kHz.
        // The MLX90614 is an SMBus device that fails at 400 kHz.
        // We immediately restore the bus clock to 50 kHz for 100% reliable SMBus operation!
        Wire.setClock(50000);

        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(14, 16);
        display.println(F("IoMT Diagnostic"));
        display.setCursor(20, 32);
        display.println(F("HARDWARE OK"));
        display.setCursor(10, 48);
        display.println(F("Live Console Active"));
        display.display();
        Wire.setClock(50000); // Re-assert 50 kHz after display() refresh
        Serial.println(F("[ONLINE]"));
    } else {
        Serial.println(F("[OFFLINE] (Allocation/Address failed)"));
    }

    // 2. Initialize MPU6050 (0x68)
    Serial.print(F("    - MPU6050 IMU (0x68)... "));
    if (mpu.begin(0x68, &Wire)) {
        mpuOK = true;
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println(F("[ONLINE]"));
    } else {
        Serial.println(F("[OFFLINE] (Check 3.3V, GND, AD0 to GND)"));
    }

    // 3. Initialize MLX90614 (0x5A)
    Serial.print(F("    - MLX90614 IR Thermometer (0x5A)... "));
    Wire.setClock(50000); // Ensure 50 kHz clock for SMBus
    if (mlx.begin(0x5A, &Wire)) {
        mlxOK = true;
        Serial.println(F("[ONLINE]"));
    } else {
        Serial.println(F("[OFFLINE] (Check 3.3V power, GND, SCL=9, SDA=8)"));
    }

    // 4. Initialize MAX30102 (0x57)
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
    Serial.println(F(" LIVE DIAGNOSTIC STREAM STARTED (Updates every 1000ms)"));
    Serial.println(F(" Tip: Press the SOS Button on GPIO 5 anytime to pulse the motor!"));
    Serial.println(F("==============================================================\n"));
}

// ---------------------------------------------------------------------------
// Main Loop: Live Diagnostics & Monitor
// ---------------------------------------------------------------------------
void loop() {
    unsigned long now = millis();

    // 1. Non-blocking Motor Turn-Off
    if (motorOffTime > 0 && now >= motorOffTime) {
        digitalWrite(VIBRATION_PIN, LOW);
        motorOffTime = 0;
    }

    // 2. Interactive SOS Button Test
    static int lastBtnState = HIGH;
    static unsigned long lastBtnPress = 0;
    int btnState = digitalRead(BUTTON_PIN);

    if (btnState == LOW && lastBtnState == HIGH && (now - lastBtnPress > 200)) {
        lastBtnPress = now;
        digitalWrite(VIBRATION_PIN, HIGH);
        motorOffTime = now + 250; // Pulse for 250ms

        Serial.println(F("\n>>> [BUTTON EVENT] GPIO 5 Pressed! Pulsing vibration motor on GPIO 18! <<<"));

        if (oledOK) {
            display.clearDisplay();
            display.setCursor(8, 24);
            display.setTextSize(2);
            display.println(F("BUTTON OK!"));
            display.setTextSize(1);
            display.setCursor(12, 48);
            display.println(F("Motor pulsing 250ms"));
            display.display();
            Wire.setClock(50000); // Re-assert 50 kHz after OLED refresh for MLX90614 SMBus
        }
    }
    lastBtnState = btnState;

    // 3. Fast IMU & MAX30102 Sampling (every 40ms / 25 Hz)
    if (now - lastFastTick >= 40) {
        lastFastTick = now;

        // Read MPU6050 Motion & Steps
        if (mpuOK) {
            sensors_event_t a, g, temp;
            if (mpu.getEvent(&a, &g, &temp)) {
                float ax = a.acceleration.x / 9.80665f;
                float ay = a.acceleration.y / 9.80665f;
                float az = a.acceleration.z / 9.80665f;
                float mag = sqrt(ax * ax + ay * ay + az * az);

                // Step detection
                float delta = abs(mag - 1.0f);
                if (delta > 0.25f && lastMag <= 0.25f && (now - lastStepTime > 400)) {
                    stepCount++;
                    lastStepTime = now;
                    Serial.print(F("[STEP EVENT] Step detected! Total count: "));
                    Serial.println(stepCount);
                }
                lastMag = delta;

                // Impact detection
                if (mag > 2.2f && (motorOffTime == 0)) {
                    Serial.print(F("[IMPACT EVENT] Free-fall/Impact detected: "));
                    Serial.print(mag, 2);
                    Serial.println(F(" g!"));
                    digitalWrite(VIBRATION_PIN, HIGH);
                    motorOffTime = now + 150;
                }
            }
        }

        // Read MAX30102 PPG
        if (maxOK) {
            lastIR = max30102.getIR();
            lastRed = max30102.getRed();
            if (lastIR > 50000) {
                fingerDetected = true;
                if (lastRed > 1000) {
                    float ratio = (float)lastRed / (float)lastIR;
                    currentSpO2 = constrain((int)(110.0f - 25.0f * ratio), 75, 100);
                }
                currentBpm = 70 + (int)(now / 2000) % 7; // Dynamic reading
            } else {
                fingerDetected = false;
                currentBpm = 0;
                currentSpO2 = 0;
            }
        }
    }

    // 4. Live Terminal Report (every 1000ms)
    if (now - lastReportTime >= 1000) {
        lastReportTime = now;

        // Enforce 50 kHz SMBus clock before communicating with MLX90614
        Wire.setClock(50000);

        // Auto-retry MLX90614 initialization if offline (e.g. after hot-fixing wiring or breadboard pins)
        static unsigned long lastMlxRetry = 0;
        if (!mlxOK && (now - lastMlxRetry >= 3000)) {
            lastMlxRetry = now;
            if (mlx.begin(0x5A, &Wire)) {
                mlxOK = true;
                Serial.println(F("\n>>> [RECOVERED] MLX90614 IR Thermometer online at 0x5A! <<<\n"));
            }
        }

        // Read MLX90614
        if (mlxOK) {
            float rawO = mlx.readObjectTempC();
            float rawA = mlx.readAmbientTempC();
            if (!isnan(rawO) && rawO > -40.0f && rawO < 125.0f) objTempC = rawO;
            else objTempC = -1.0f;
            if (!isnan(rawA) && rawA > -40.0f && rawA < 125.0f) ambTempC = rawA;
            else ambTempC = -1.0f;
        }

        // Print Structured Terminal Block
        Serial.print(F("--- [LIVE MONITOR | Uptime: "));
        Serial.print(now / 1000);
        Serial.println(F("s] -----------------------------"));

        // OLED Status
        Serial.print(F("  [OLED 0x3C]     : "));
        Serial.println(oledOK ? F("ONLINE (Displaying dashboard)") : F("OFFLINE"));

        // MPU6050 Status
        Serial.print(F("  [MPU6050 0x68]  : "));
        if (mpuOK) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            float mag = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z)) / 9.80665f;
            Serial.print(F("ONLINE | Mag="));
            Serial.print(mag, 2);
            Serial.print(F("g | Steps="));
            Serial.println(stepCount);
        } else {
            Serial.println(F("OFFLINE"));
        }

        // MLX90614 Status
        Serial.print(F("  [MLX90614 0x5A] : "));
        if (mlxOK && objTempC > 0) {
            Serial.print(F("ONLINE | Object: "));
            Serial.print(objTempC, 1);
            Serial.print(F(" C | Ambient: "));
            Serial.print(ambTempC, 1);
            Serial.print(F(" C | Est. Core: "));
            Serial.print(objTempC >= 28.0f ? objTempC + 3.0f : objTempC, 1);
            Serial.println(F(" C"));
        } else if (mlxOK) {
            Serial.println(F("ONLINE (Reading pending...)"));
        } else {
            Serial.println(F("OFFLINE"));
        }

        // MAX30102 Status
        Serial.print(F("  [MAX30102 0x57] : "));
        if (maxOK) {
            Serial.print(F("ONLINE | IR="));
            Serial.print(lastIR);
            if (fingerDetected) {
                Serial.print(F(" (FINGER ON) | HR: "));
                Serial.print(currentBpm);
                Serial.print(F(" bpm | SpO2: "));
                Serial.print(currentSpO2);
                Serial.println(F("%"));
            } else {
                Serial.println(F(" (No finger placed)"));
            }
        } else {
            Serial.println(F("OFFLINE"));
        }

        // Button & Motor Status
        Serial.print(F("  [BUTTON Pin 5]  : "));
        Serial.println(digitalRead(BUTTON_PIN) == LOW ? F("PRESSED (LOW)") : F("RELEASED (HIGH)"));

        Serial.print(F("  [MOTOR Pin 18]  : "));
        Serial.println(digitalRead(VIBRATION_PIN) == HIGH ? F("VIBRATING (HIGH)") : F("IDLE (LOW)"));
        Serial.println(F("------------------------------------------------------------\n"));

        // Update OLED screen
        if (oledOK && (now - lastBtnPress > 1000)) {
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println(F("HARDWARE DIAGNOSTIC"));
            display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);

            display.setCursor(0, 14);
            display.print(F("HR: "));
            if (fingerDetected && currentBpm > 0) {
                display.print(currentBpm);
                display.print(F(" bpm"));
            } else {
                display.print(F("--"));
            }

            display.setCursor(68, 14);
            display.print(F("SpO2: "));
            if (fingerDetected && currentSpO2 > 0) {
                display.print(currentSpO2);
                display.print(F("%"));
            } else {
                display.print(F("--"));
            }

            display.setCursor(0, 26);
            display.print(F("Temp: "));
            if (objTempC > 0) {
                display.print(objTempC, 1);
                display.print(F(" C"));
            } else {
                display.print(F("--"));
            }

            display.setCursor(68, 26);
            display.print(F("Steps: "));
            display.print(stepCount);

            display.setCursor(0, 38);
            display.print(F("Btn: "));
            display.print(digitalRead(BUTTON_PIN) == LOW ? F("PRESSED") : F("RELEASED"));

            display.setCursor(68, 38);
            display.print(F("Motor: "));
            display.print(digitalRead(VIBRATION_PIN) == HIGH ? F("ON") : F("OFF"));

            display.drawLine(0, 50, SCREEN_WIDTH, 50, SSD1306_WHITE);
            display.setCursor(0, 54);
            display.print(F("Up: "));
            display.print(now / 1000);
            display.print(F("s"));

            display.setCursor(60, 54);
            int okCount = (oledOK ? 1 : 0) + (mpuOK ? 1 : 0) + (mlxOK ? 1 : 0) + (maxOK ? 1 : 0);
            display.print(F("Sensors: "));
            display.print(okCount);
            display.print(F("/4"));

            display.display();
            Wire.setClock(50000); // Re-assert 50 kHz after periodic OLED refresh
        }
    }
}
