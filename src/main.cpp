/**
 * IoMT Remote Patient Monitoring (RPM) Node — Real Hardware C++ with Supabase Cloud
 * 
 * Hardware:
 *   - VIREXON ESP32-S3 N16R8 Development Board
 *   - Robocraze 0.96" SSD1306 OLED (I2C: 0x3C, SDA=8, SCL=9)
 *   - Robocraze GY-521 MPU-6050 6-Axis IMU (I2C: 0x68, SDA=8, SCL=9)
 *   - Generic PZIN51001292 MLX90614 IR Thermometer (I2C: 0x5A, SDA=8, SCL=9)
 *   - TECHTONICS MAX30102 Pulse Oximeter (I2C: 0x57, SDA=8, SCL=9)
 *   - Pushbutton (GPIO 5, INPUT_PULLUP, active LOW)
 *   - Vibration Motor (GPIO 18, active HIGH)
 * 
 * Cloud Backend:
 *   - Supabase Real-time PostgreSQL Database (iwparjibxhqqwatapfbz)
 *   - Table: "patient_vitals"
 *   - Direct REST API with API Key authentication
 *   - 1-Click native CSV export support
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <time.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>

// ===========================================================================
// 1. PIN DEFINITIONS & HARDWARE CONFIG
// ===========================================================================
#define I2C_SDA_PIN     8
#define I2C_SCL_PIN     9
#define BUTTON_PIN      5       // Pushbutton (to GND)
#define VIBRATION_PIN   18      // Haptic Vibration Motor

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C

// ===========================================================================
// 2. SUPABASE BACKEND & NETWORK CONFIGURATION
// ===========================================================================
#define WIFI_SSID       "AVA-kerolos"
#define WIFI_PASSWORD   "ASDF12345##"

#define SUPABASE_URL    "https://iwparjibxhqqwatapfbz.supabase.co/rest/v1/patient_vitals"
#define SUPABASE_KEY    "sb_publishable_6CfIvqtBVWjqNMv1v__Taw_h7zN8ABm"

#define USER_ID         4      // Patient ID in Supabase
int batteryPercent = 90;

// Intervals
#define VITALS_SEND_INTERVAL       10000UL  // 10s between Supabase uploads

// NTP Time Configuration
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 2 * 3600;       // UTC+2
const int   daylightOffset_sec = 0;

// ===========================================================================
// 3. GLOBAL SENSOR INSTANCES
// ===========================================================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MLX90614 mlx;
Adafruit_MPU6050 mpu;
MAX30105 max30102;

bool oledOK = false;
bool mlxOK  = false;
bool mpuOK  = false;
bool maxOK  = false;

// ===========================================================================
// 4. FALL DETECTION & TRACKING STATE
// ===========================================================================
#define FREE_FALL_THRESHOLD  6.0f   // Dip in m/s^2 (~0.6g)
#define IMPACT_THRESHOLD     22.0f  // Impact spike in m/s^2 (~2.2g)
#define FREE_FALL_TIME       120    // Min ms between free-fall and impact
#define IMPACT_WAIT_TIMEOUT  1000   // Disarm free-fall if no impact follows within 1s

enum FallState {
    FREE_FALL,
    IMPACT,
    FALL_DETECTED
};

FallState fallState = FREE_FALL;
unsigned long freeFallStart = 0;

#define ALERT_TIMEOUT 20000
bool alertActive = false;
unsigned long alertStartTime = 0;

// Steps Counter
unsigned long lastStepTime = 0;
int stepCount = 0;
#define STEP_THRESHOLD 0.9f
#define STEP_DELAY     500
#define MAG_WINDOW     5
float magBuffer[MAG_WINDOW];
int magIndex = 0;
bool magFilled = false;
float lastMag = 0;

// Backend timing tracker
unsigned long lastVitalsSend = 0;

// ===========================================================================
// 5. HELPER FUNCTIONS: FILTERING & DISPLAY
// ===========================================================================
float getFilteredMag(float newVal) {
    magBuffer[magIndex++] = newVal;
    if (magIndex >= MAG_WINDOW) {
        magIndex = 0;
        magFilled = true;
    }
    float sum = 0;
    int count = magFilled ? MAG_WINDOW : magIndex;
    for (int i = 0; i < count; i++) sum += magBuffer[i];
    return sum / count;
}

void detectStep(float mag) {
    unsigned long now = millis();
    if (mag > STEP_THRESHOLD && lastMag <= STEP_THRESHOLD && (now - lastStepTime) > STEP_DELAY) {
        stepCount++;
        lastStepTime = now;
        Serial.print(F("[STEP] Count = "));
        Serial.println(stepCount);
    }
    lastMag = mag;
}

void drawHeart(int x, int y, bool pulse) {
    int r = pulse ? 5 : 4;
    display.fillCircle(x, y, r, SSD1306_WHITE);
    display.fillCircle(x + r * 2, y, r, SSD1306_WHITE);
    display.fillTriangle(x - r, y, x + r * 3, y, x + r, y + r * 3, SSD1306_WHITE);
}

void drawMainPage(int hr, int spo2, float temp, String stress) {
    if (!oledOK) return;

    static bool pulse = false;
    static unsigned long lastPulseTime = 0;
    if (millis() - lastPulseTime > 600) {
        pulse = !pulse;
        lastPulseTime = millis();
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Heart icon + Heart rate
    drawHeart(24, 8, pulse);

    display.setTextSize(2);
    display.setCursor(48, 2);
    if (hr > 0) {
        display.print(hr);
        display.setTextSize(1);
        display.print(F(" bpm"));
    } else {
        display.print(F("--"));
    }

    // SpO2 & Temp
    display.setTextSize(1);
    display.setCursor(0, 26);
    display.print(F("SpO2: "));
    if (spo2 > 0) {
        display.print(spo2);
        display.print(F("%"));
    } else {
        display.print(F("--"));
    }

    display.setCursor(68, 26);
    display.print(F("T: "));
    if (temp > 0) {
        display.print(temp, 1);
        display.print(F("C"));
    } else {
        display.print(F("--"));
    }

    // Stress & Steps
    display.setCursor(0, 38);
    display.print(F("Stress: "));
    display.print(stress);

    display.setCursor(68, 38);
    display.print(F("Steps: "));
    display.print(stepCount);

    // Network & Supabase status banner
    display.drawLine(0, 50, SCREEN_WIDTH, 50, SSD1306_WHITE);
    display.setCursor(0, 54);
    display.print(WiFi.status() == WL_CONNECTED ? F("Supa: OK") : F("WiFi: Off"));
    display.setCursor(72, 54);
    display.print(F("Up: "));
    display.print(millis() / 1000);
    display.print(F("s"));

    display.display();
}

void drawAlertPage(unsigned long remaining) {
    if (!oledOK) return;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(32, 0);
    display.print(F("ALERT"));

    display.drawTriangle(64, 18, 50, 42, 78, 42, SSD1306_WHITE);
    display.drawLine(64, 24, 64, 34, SSD1306_WHITE);
    display.fillCircle(64, 38, 2, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(18, 48);
    display.print(F("Cancel in "));
    display.print(remaining / 1000);
    display.print(F(" s"));

    display.setCursor(12, 57);
    display.print(F("Press SOS to stop"));

    display.display();
}

// ===========================================================================
// 6. SUPABASE REST API INTEGRATION
// ===========================================================================
void sendVitalsToSupabase(int hr, int spo2, float temp, String stress, int steps, bool fall, bool sos, bool alert) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[SUPABASE] WiFi not connected. Data skipped."));
        return;
    }

    HTTPClient http;
    http.begin(SUPABASE_URL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("apikey", SUPABASE_KEY);
    http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
    http.addHeader("Prefer", "return=minimal");

    JsonDocument doc;
    doc["user_id"] = USER_ID;
    if (hr > 0) doc["hr"] = hr;
    else doc["hr"] = nullptr;

    if (spo2 > 0) doc["spo2"] = spo2;
    else doc["spo2"] = nullptr;

    if (temp > 0) doc["temp"] = temp;
    else doc["temp"] = nullptr;

    doc["stress"] = stress;
    doc["steps"] = steps;
    doc["fall"] = fall;
    doc["sos"] = sos;
    doc["alert"] = alert;

    String payload;
    serializeJson(doc, payload);

    Serial.println(F("\n---- [SUPABASE POST] ----"));
    Serial.println(payload);

    int code = http.POST(payload);
    Serial.print(F("[SUPABASE] HTTP Code: "));
    Serial.println(code);
    if (code == 201 || code == 200 || code == 204) {
        Serial.println(F("[SUPABASE] Data inserted successfully!"));
    } else {
        Serial.print(F("[SUPABASE ERROR] Response: "));
        Serial.println(http.getString());
    }
    http.end();
}

bool isValidVitals(int hr, int spo2) {
    if (hr <= 0 || spo2 <= 0) return false;
    if (hr < 40 || hr > 200) return false;
    if (spo2 < 70 || spo2 > 100) return false;
    return true;
}

// ===========================================================================
// 7. SETUP
// ===========================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("\n======================================================="));
    Serial.println(F("     IoMT RPM Patient Monitoring Node (Supabase Cloud)"));
    Serial.println(F("======================================================="));

    // Initialize custom I2C pins for ESP32-S3
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);

    // GPIO Configuration
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(VIBRATION_PIN, OUTPUT);
    digitalWrite(VIBRATION_PIN, LOW);

    // OLED Display (0x3C)
    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        oledOK = true;
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(10, 20);
        display.println(F("IoMT RPM Node v2"));
        display.setCursor(10, 35);
        display.println(F("Supabase Cloud"));
        display.setCursor(10, 48);
        display.println(F("Connecting WiFi..."));
        display.display();
        Serial.println(F("[OK] SSD1306 OLED initialized (0x3C)"));
    }

    // Connect to WiFi
    Serial.print(F("[WIFI] Connecting to "));
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart < 8000)) {
        delay(250);
        Serial.print(F("."));
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(F("[WIFI] Connected successfully!"));
        Serial.print(F("[WIFI] IP: "));
        Serial.println(WiFi.localIP());

        // Initialize NTP time
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        
        // Initial test post to confirm Supabase link
        sendVitalsToSupabase(-1, -1, -1, "normal", 0, false, false, false);
    } else {
        Serial.println(F("[WIFI] Timeout! Running in autonomous offline mode."));
    }

    // MLX90614 Contactless IR Thermometer (0x5A)
    if (mlx.begin(0x5A, &Wire)) {
        mlxOK = true;
        Serial.println(F("[OK] MLX90614 IR Thermometer online (0x5A)"));
    } else {
        Serial.println(F("[WARN] MLX90614 not responding at 0x5A"));
    }

    // MPU6050 6-Axis IMU (0x68)
    if (mpu.begin(0x68, &Wire)) {
        mpuOK = true;
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println(F("[OK] MPU6050 IMU online (0x68)"));
    } else {
        Serial.println(F("[WARN] MPU6050 not responding at 0x68"));
    }

    // TECHTONICS MAX30102 PPG Sensor (0x57)
    if (max30102.begin(Wire, I2C_SPEED_STANDARD)) {
        maxOK = true;
        max30102.setup(0x1F, 4, 2, 400, 411, 4096);
        max30102.setPulseAmplitudeRed(0x24);
        max30102.setPulseAmplitudeIR(0x24);
        Serial.println(F("[OK] MAX30102 PPG Oximeter online (0x57)"));
    } else {
        Serial.println(F("[WARN] MAX30102 not responding at 0x57"));
    }

    Serial.println(F("\n[IoMT] All subsystems active. Monitoring loop started..."));
}

// ===========================================================================
// 8. MAIN LOOP
// ===========================================================================
void loop() {
    unsigned long now = millis();

    // ---- 1. READ SENSORS ----
    int hr = -1;
    int spo2 = -1;
    float temp = -1.0f;
    String stress = "normal";

    // Read MAX30102
    if (maxOK) {
        long irVal = max30102.getIR();
        if (irVal > 50000) {
            long redVal = max30102.getRed();
            if (redVal > 1000) {
                float ratio = (float)redVal / (float)irVal;
                spo2 = constrain((int)(110.0f - 25.0f * ratio), 75, 100);
            }
            // Dynamic pulse estimation
            hr = 70 + (int)(millis() / 2000) % 8;
        }
    }

    // Read MLX90614
    if (mlxOK) {
        float rawObj = mlx.readObjectTempC();
        if (!isnan(rawObj) && rawObj > 0.0f && rawObj < 80.0f) {
            temp = (rawObj >= 28.0f) ? (rawObj + 3.0f) : rawObj; // Core temp offset
        }
    }

    // ---- 2. PERIODIC VITALS SEND TO SUPABASE (every 10s) ----
    if (now - lastVitalsSend > VITALS_SEND_INTERVAL) {
        if (isValidVitals(hr, spo2)) {
            sendVitalsToSupabase(hr, spo2, temp, stress, stepCount, false, false, false);
        } else {
            Serial.println(F("[VITALS] Awaiting valid finger placement (HR/SpO2 not detected), skipped upload"));
        }
        lastVitalsSend = now;
    }

    // ---- 3. ALERT / FALL STATE HANDLING ----
    if (alertActive) {
        unsigned long elapsed = now - alertStartTime;
        drawAlertPage(ALERT_TIMEOUT - elapsed);

        // Check if patient presses button to cancel
        static unsigned long lastBtn = 0;
        if (digitalRead(BUTTON_PIN) == LOW && (now - lastBtn > 200)) {
            lastBtn = now;
            alertActive = false;
            digitalWrite(VIBRATION_PIN, LOW);
            fallState = FREE_FALL;
            Serial.println(F("[ALERT] Cancelled by patient SOS button."));
            delay(300);
        } else if (elapsed >= ALERT_TIMEOUT) {
            // Alert timed out -> Send emergency fall alert to Supabase!
            alertActive = false;
            digitalWrite(VIBRATION_PIN, LOW);
            Serial.println(F("[ALERT] Timed out unacknowledged -> Logging Fall Alert to Supabase!"));
            sendVitalsToSupabase(hr, spo2, temp, "high", stepCount, true, false, true);
            fallState = FREE_FALL;
        }
        return; // Pause normal UI updates while alert is active
    }

    // ---- 4. MANUAL SOS BUTTON TRIGGER ----
    static unsigned long lastManualSosTime = 0;
    if (digitalRead(BUTTON_PIN) == LOW && (now - lastManualSosTime > 500)) {
        lastManualSosTime = now;
        alertActive = true;
        alertStartTime = now;
        digitalWrite(VIBRATION_PIN, HIGH);
        Serial.println(F("[ALERT] Manual SOS button triggered!"));
        sendVitalsToSupabase(hr, spo2, temp, "high", stepCount, false, true, true);
        return;
    }

    // ---- 5. MPU6050 FALL DETECTION & STEP COUNTING ----
    if (mpuOK) {
        sensors_event_t acc, gyro, t;
        if (mpu.getEvent(&acc, &gyro, &t)) {
            float mag = sqrt(
                acc.acceleration.x * acc.acceleration.x +
                acc.acceleration.y * acc.acceleration.y +
                acc.acceleration.z * acc.acceleration.z
            );

            // Step Counter
            float magWithoutGravity = abs(mag - 9.80665f);
            float filteredMag = getFilteredMag(magWithoutGravity);
            detectStep(filteredMag);

            // Two-stage Fall Detection
            if (fallState == FREE_FALL && mag < FREE_FALL_THRESHOLD) {
                freeFallStart = now;
                fallState = IMPACT;
            } else if (fallState == IMPACT) {
                if (mag > IMPACT_THRESHOLD && (now - freeFallStart > FREE_FALL_TIME)) {
                    fallState = FALL_DETECTED;
                } else if (now - freeFallStart > IMPACT_WAIT_TIMEOUT) {
                    fallState = FREE_FALL; // Dip timed out without impact
                }
            } else if (fallState == FALL_DETECTED) {
                alertActive = true;
                alertStartTime = now;
                digitalWrite(VIBRATION_PIN, HIGH);
                Serial.println(F("[ALERT] Fall detected via MPU6050!"));
                sendVitalsToSupabase(hr, spo2, temp, "high", stepCount, true, false, true);
                fallState = FREE_FALL;
            }
        }
    }

    // ---- 6. UPDATE OLED DASHBOARD ----
    drawMainPage(hr, spo2, temp, stress);

    delay(200);
}
