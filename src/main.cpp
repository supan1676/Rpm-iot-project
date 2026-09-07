#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHTesp.h>

// I2C Pinout for ESP32-S3
#define I2C_SDA 8
#define I2C_SCL 9

// Digital & Analog Pins
#define DHTPIN       5
#define POT_HR_PIN   4    // Rotary Pot -> Simulated Heart Rate
#define POT_SPO2_PIN 6    // Slide Pot  -> Simulated SpO2 / Respiration
#define PIR_PIN      7    // Bed mobility / exit
#define SOS_PIN      10   // Emergency Pushbutton
#define TRIG_PIN     15   // Ultrasonic Trigger
#define ECHO_PIN     16   // Ultrasonic Echo
#define BUZZER_PIN   18   // Clinical Alert

// Display Config
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;
DHTesp dht;

// Fall threshold (~2.5G total acceleration)
const float FALL_THRESHOLD = 24.5;

// DHT22 reads are slow (~2s minimum interval), track separately
unsigned long lastDHTRead = 0;
const unsigned long DHT_INTERVAL = 2500; // 2.5 seconds between reads
float lastTempC = NAN;
float lastHumidity = NAN;

// Sensor init status for self-test
bool oledOK = false;
bool mpuOK  = false;

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize Custom I2C for ESP32-S3
  Wire.begin(I2C_SDA, I2C_SCL);

  // Pin Modes
  pinMode(SOS_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // DHT22 init (using DHTesp library — required for Wokwi ESP32 compatibility)
  dht.setup(DHTPIN, DHTesp::DHT22);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("[ERROR] SSD1306 allocation failed"));
  } else {
    oledOK = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 20);
    display.println(F("IoMT RPM Node"));
    display.setCursor(10, 35);
    display.println(F("Initializing..."));
    display.display();
  }

  // MPU6050 init
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println(F("[WARN] MPU6050 not detected. Check I2C wiring."));
  } else {
    mpuOK = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Self-test summary
  Serial.println(F(""));
  Serial.println(F("[IoMT] Self-Test Results:"));
  Serial.print(F("  OLED   : ")); Serial.println(oledOK  ? "OK" : "FAIL");
  Serial.print(F("  MPU6050: ")); Serial.println(mpuOK   ? "OK" : "FAIL");
  Serial.println(F("  DHT22  : OK (first read in ~2s)"));
  Serial.println(F("  Buzzer : Ready"));
  Serial.println(F("  HC-SR04: Ready"));
  Serial.println(F(""));
  Serial.println(F("[IoMT] System Initialized. Streaming telemetry..."));

  delay(1000); // Let splash screen show briefly
}

float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000); // 25ms timeout
  if (duration == 0) return -1.0;
  float dist = (duration * 0.0343) / 2.0;
  // Cap unrealistic readings (beyond HC-SR04 max range ~400cm)
  if (dist > 400.0) return -1.0;
  return dist;
}

void loop() {
  // 1. Environmental & Body Temperature (read every 2.5s, DHT22 is slow)
  unsigned long now = millis();
  if (now - lastDHTRead >= DHT_INTERVAL) {
    lastDHTRead = now;
    TempAndHumidity data = dht.getTempAndHumidity();
    // Debug: print raw DHT status on every read attempt
    Serial.print("[DHT] status=");
    Serial.print(dht.getStatusString());
    Serial.print(" temp=");
    Serial.print(data.temperature);
    Serial.print(" hum=");
    Serial.println(data.humidity);
    // Use isnan() check directly (Wokwi recommended pattern)
    if (!isnan(data.temperature)) lastTempC = data.temperature;
    if (!isnan(data.humidity))    lastHumidity = data.humidity;
  }
  float tempC = lastTempC;
  float humidity = lastHumidity;

  // 2. Simulated Hemodynamic Vitals via Potentiometers
  int rawHR = analogRead(POT_HR_PIN);
  int rawSpO2 = analogRead(POT_SPO2_PIN);
  int heartRate = map(rawHR, 0, 4095, 45, 160);       // BPM (full range preserved)
  int spO2 = map(rawSpO2, 0, 4095, 85, 100);          // Oxygen Saturation %

  // 3. Fall Detection via IMU Vector Magnitude
  sensors_event_t a, g, mpuTemp;
  bool mpuActive = mpu.getEvent(&a, &g, &mpuTemp);
  float totalAccel = 9.8;
  bool fallDetected = false;

  if (mpuActive) {
    totalAccel = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
    if (totalAccel > FALL_THRESHOLD) {
      fallDetected = true;
    }
  }

  // 4. Presence & Emergency Triggers
  bool bedMotion = digitalRead(PIR_PIN);
  bool sosActive = (digitalRead(SOS_PIN) == LOW);
  float bedDistance = getUltrasonicDistance();

  // 5. Clinical Threshold Rule Engine
  bool hrCritical = (heartRate > 120 || heartRate < 50);
  bool spo2Critical = (spO2 < 90);
  bool tempCritical = (!isnan(tempC) && tempC > 38.5);
  bool vitalsCritical = hrCritical || spo2Critical || tempCritical;
  bool emergencyAlert = sosActive || fallDetected || vitalsCritical;

  // 6. Actuator Logic (Pulsing Buzzer — 500ms on/off)
  if (emergencyAlert) {
    if ((millis() / 500) % 2 == 0)
      digitalWrite(BUZZER_PIN, HIGH);
    else
      digitalWrite(BUZZER_PIN, LOW);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  // 7. Telemetry Output (JSON over Serial)
  Serial.print("{\"bpm\":");
  Serial.print(heartRate);
  Serial.print(",\"spo2\":");
  Serial.print(spO2);
  Serial.print(",\"temp_c\":");
  if (isnan(tempC)) Serial.print("null");
  else Serial.print(tempC, 1);
  Serial.print(",\"humidity_pct\":");
  if (isnan(humidity)) Serial.print("null");
  else Serial.print(humidity, 1);
  Serial.print(",\"motion\":");
  Serial.print(bedMotion ? "true" : "false");
  Serial.print(",\"bed_dist_cm\":");
  if (bedDistance < 0) Serial.print("null");
  else Serial.print(bedDistance, 1);
  Serial.print(",\"fall\":");
  Serial.print(fallDetected ? "true" : "false");
  Serial.print(",\"sos\":");
  Serial.print(sosActive ? "true" : "false");
  Serial.print(",\"alert\":");
  Serial.print(emergencyAlert ? "true" : "false");
  Serial.print(",\"uptime_s\":");
  Serial.print(millis() / 1000);
  Serial.println("}");

  // 8. Update Local OLED Display
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (emergencyAlert) {
    display.println(F("!! CRITICAL ALERT !!"));
    if (sosActive)         display.println(F("CALL: SOS Triggered"));
    else if (fallDetected) display.println(F("CALL: Fall Detected"));
    else if (hrCritical)   display.println(F("CALL: HR Abnormal"));
    else if (spo2Critical) display.println(F("CALL: Low SpO2"));
    else if (tempCritical) display.println(F("CALL: High Temp"));
  } else {
    display.println(F("PATIENT MONITOR: OK"));
    display.println(F("---------------------"));
  }

  display.setCursor(0, 22);
  display.print(F("HR  : ")); display.print(heartRate); display.println(F(" BPM"));
  display.print(F("SpO2: ")); display.print(spO2); display.println(F(" %"));
  display.print(F("Temp: "));
  if (isnan(tempC)) display.println(F("-- C"));
  else { display.print(tempC, 1); display.println(F(" C")); }

  display.print(F("Bed : "));
  display.println(bedMotion ? F("Moving") : F("Resting"));

  display.display();

  delay(1000);
}