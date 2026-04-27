/*
  HealthMonitor ESP32 Firmware — v2.1
  ─────────────────────────────────────────────────────────────────────────────
  Fix in v2.1:
    • PTT range corrected to 300–1500 ms (covers 40–200 BPM)
    • Peak window rolls before BPM is calculated (single source of truth)
    • Out-of-range PTT candidate keeps last valid value instead of -1
    • beatsPerMinute derived from same inter-beat delta as PTT

  Wiring (ESP32 DevKit):
  ┌─────────────┬──────────┐
  │ MAX30102 SDA│ GPIO 21  │
  │ MAX30102 SCL│ GPIO 22  │
  │ MAX30102 VCC│ 3.3 V    │
  │ MAX30102 GND│ GND      │
  │ DS18B20 DQ  │ GPIO 4   │  4.7 kΩ pull-up to 3.3 V
  │ DS18B20 VCC │ 3.3 V    │
  │ DS18B20 GND │ GND      │
  └─────────────┴──────────┘

  Libraries (Arduino Library Manager):
    - SparkFun MAX3010x Pulse and Proximity Sensor Library
    - OneWire
    - DallasTemperature
    - ArduinoJson (v6+)
*/

// ─── User configuration ───────────────────────────────────────────────────────
#define WIFI_SSID      "YOUR_SSID"
#define WIFI_PASSWORD  "YOUR_PASSWORD"
#define API_HOST       "your-app.onrender.com"  // ← no https://
#define API_ENDPOINT   "/readings"
#define DEVICE_ID      "esp32-01"
// ─────────────────────────────────────────────────────────────────────────────

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

#include <OneWire.h>
#include <DallasTemperature.h>

// ─── Pins ─────────────────────────────────────────────────────────────────────
#define DS18B20_PIN 4

OneWire           oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
MAX30105          particleSensor;

// ─── SpO2 buffers ─────────────────────────────────────────────────────────────
#define BUFFER_SIZE 100
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

int32_t spo2      = 0;  int8_t validSPO2 = 0;
int32_t heartRate = 0;  int8_t validHR   = 0;

// ─── Beat / PTT state ─────────────────────────────────────────────────────────
#define RATE_SIZE 4
byte  rates[RATE_SIZE]  = {0};
byte  rateSpot          = 0;
float beatsPerMinute    = 0.0f;
int   beatAvg           = 0;

// Two consecutive IR peak timestamps (milliseconds)
unsigned long peakTime1 = 0;
unsigned long peakTime2 = 0;

// Last valid PTT in ms. Stays at -1 until two beats have been captured.
float pttMs = -1.0f;

// ─── Timing ───────────────────────────────────────────────────────────────────
#define PUSH_INTERVAL_MS 1000
unsigned long lastPushTime = 0;

// ─── Forward declarations ─────────────────────────────────────────────────────
void  connectWiFi();
void  initMAX30102();
void  collectBatch();
float readDS18B20();
void  pushToAPI(float ds_c, float max_c, int32_t hr, int32_t spo2Val,
                bool hrOk, bool spo2Ok, float ptt);

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== HealthMonitor v2.1 ===");
  connectWiFi();
  ds18b20.begin();
  Serial.printf("DS18B20 devices: %d\n", ds18b20.getDeviceCount());
  initMAX30102();
  collectBatch();
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_SIZE, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHR
  );
  Serial.println("Ready — streaming.");
}

void loop() {
  // ── 1. Slide buffer: drop oldest 25, keep newest 75 ──────────────────────
  for (byte i = 25; i < BUFFER_SIZE; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25]  = irBuffer[i];
  }

  // ── 2. Collect 25 fresh samples ──────────────────────────────────────────
  for (byte i = 75; i < BUFFER_SIZE; i++) {
    while (particleSensor.available() == false)
      particleSensor.check();

    long irValue = particleSensor.getIR();

    // ── Beat detection + PTT ────────────────────────────────────────────────
    if (checkForBeat(irValue)) {
      unsigned long now = millis();

      // Roll the peak window FIRST so both PTT and BPM use the same delta
      peakTime1 = peakTime2;
      peakTime2 = now;

      if (peakTime1 != 0 && peakTime2 > peakTime1) {
        float ibi = (float)(peakTime2 - peakTime1);  // inter-beat interval ms

        // ── PTT: accept 300–1500 ms (40–200 BPM range) ──────────────────
        // If candidate is out of range, keep the last valid pttMs — do NOT
        // reset to -1, which would cause the API to drop the field entirely.
        if (ibi >= 300.0f && ibi <= 1500.0f) {
          pttMs = ibi;
        }

        // ── BPM from same interval ───────────────────────────────────────
        beatsPerMinute = 60000.0f / ibi;
        if (beatsPerMinute > 20 && beatsPerMinute < 255) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }
    }
    // ────────────────────────────────────────────────────────────────────────

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = irValue;
    particleSensor.nextSample();
  }

  // ── 3. Recalculate SpO2 + HR from updated buffer ─────────────────────────
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_SIZE, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHR
  );

  // ── 4. Read temperatures ─────────────────────────────────────────────────
  float ds_c  = readDS18B20();
  float max_c = particleSensor.readTemperature();

  // ── 5. Serial debug ──────────────────────────────────────────────────────
  Serial.printf(
    "[%lums] SpO2=%d(%s) HR=%d avg=%d PTT=%.1fms DS=%.2f°C die=%.2f°C\n",
    millis(),
    spo2,      validSPO2 ? "OK" : "??",
    heartRate, beatAvg,
    pttMs,
    ds_c, max_c
  );

  // ── 6. Push to API every second ──────────────────────────────────────────
  if (millis() - lastPushTime >= PUSH_INTERVAL_MS) {
    lastPushTime = millis();
    int32_t bpm  = (beatAvg > 0) ? beatAvg : heartRate;
    pushToAPI(ds_c, max_c, bpm, spo2, validHR, validSPO2, pttMs);
  }
}

// ─── Wi-Fi ────────────────────────────────────────────────────────────────────
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++attempts > 40) {
      Serial.printf("\nFailed (status=%d). Restarting.\n", WiFi.status());
      ESP.restart();
    }
  }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
}

// ─── MAX30102 init ────────────────────────────────────────────────────────────
void initMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found! Check wiring.");
    while (1);
  }
  // ledBrightness=60, sampleAverage=4, ledMode=2(Red+IR),
  // sampleRate=100, pulseWidth=411, adcRange=4096
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  particleSensor.enableDIETEMPRDY();
  Serial.println("MAX30102 ready.");
}

// ─── Collect first 100 samples (blocking, runs once at startup) ───────────────
void collectBatch() {
  Serial.println("Collecting initial 100 samples...");
  for (byte i = 0; i < BUFFER_SIZE; i++) {
    while (particleSensor.available() == false)
      particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  Serial.println("Initial batch done.");
}

// ─── DS18B20 temperature ──────────────────────────────────────────────────────
float readDS18B20() {
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C) {
    Serial.println("WARNING: DS18B20 disconnected.");
    return -127.0f;
  }
  return t;
}

// ─── HTTPS POST → Render ──────────────────────────────────────────────────────
void pushToAPI(float ds_c, float max_c, int32_t hr, int32_t spo2Val,
               bool hrOk, bool spo2Ok, float ptt) {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi lost — reconnecting.");
    connectWiFi();
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();  // Skip TLS cert check (fine for MVP)

  HTTPClient http;
  String url = String("https://") + API_HOST + API_ENDPOINT;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(8000);

  StaticJsonDocument<320> doc;
  doc["device_id"]       = DEVICE_ID;
  doc["timestamp_ms"]    = (uint32_t)millis();
  doc["bpm"]             = hr;
  doc["bpm_valid"]       = hrOk;
  doc["spo2"]            = spo2Val;
  doc["spo2_valid"]      = spo2Ok;
  doc["temp_body_c"]     = serialized(String(ds_c,  2));
  doc["temp_body_f"]     = serialized(String(ds_c * 9.0f / 5.0f + 32.0f, 2));
  doc["temp_die_c"]      = serialized(String(max_c, 2));
  doc["finger_detected"] = (irBuffer[BUFFER_SIZE - 1] >= 50000);

  // Only send ptt_ms once we have a valid reading (> 0)
  if (ptt > 0.0f) {
    doc["ptt_ms"] = serialized(String(ptt, 1));
  }

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  Serial.printf("  → POST HTTP %d  ptt=%.1fms\n", code, ptt);
  http.end();
}
