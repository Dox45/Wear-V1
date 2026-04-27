/*
  HealthMonitor ESP32 Firmware — v2
  ─────────────────────────────────────────────────────────────────────────────
  Changes from v1:
    • Computes PTT (Pulse Transit Time) from consecutive IR waveform peaks
    • ptt_ms field added to JSON payload for server-side BP estimation
    • API_HOST updated for cloud deployment (set your Render URL below)

  Wiring: unchanged from v1 — see original header for pin table.

  Libraries required:
    - SparkFun MAX3010x Pulse and Proximity Sensor Library
    - OneWire
    - DallasTemperature
    - ArduinoJson (≥ v6)
*/

// ─── User configuration ───────────────────────────────────────────────────────
#define WIFI_SSID      "YOUR_SSID"
#define WIFI_PASSWORD  "YOUR_PASSWORD"
#define API_HOST       "your-app.onrender.com"   // ← Render URL, no https://
#define API_PORT       443                        // HTTPS on Render
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

// ─── Pins ────────────────────────────────────────────────────────────────────
#define DS18B20_PIN 4

OneWire           oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
MAX30105          particleSensor;

// ─── SpO2 buffers ────────────────────────────────────────────────────────────
#define BUFFER_SIZE 100
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

int32_t spo2      = 0;  int8_t validSPO2 = 0;
int32_t heartRate = 0;  int8_t validHR   = 0;

// ─── PBA beat state ───────────────────────────────────────────────────────────
#define RATE_SIZE 4
byte  rates[RATE_SIZE];
byte  rateSpot       = 0;
long  lastBeat       = 0;
float beatsPerMinute = 0.0f;
int   beatAvg        = 0;

// ─── PTT state ───────────────────────────────────────────────────────────────
// We store the timestamps (millis) of the last two detected IR peaks.
// PTT = time between consecutive peaks (≈ inter-beat interval proxy).
// For a true multi-site PTT you need two sensors; single-site gives PAT,
// which is still linearly correlated with BP and usable with calibration.
unsigned long peakTime1  = 0;
unsigned long peakTime2  = 0;
float         pttMs      = -1.0f;   // -1 = not yet valid

// ─── Timing ──────────────────────────────────────────────────────────────────
#define PUSH_INTERVAL_MS 1000
unsigned long lastPushTime = 0;

// ─── Helpers ─────────────────────────────────────────────────────────────────
void    connectWiFi();
void    initMAX30102();
void    collectBatch();
float   readDS18B20();
void    pushToAPI(float ds_c, float max_c, int32_t hr, int32_t spo2Val,
                  bool hrOk, bool spo2Ok, float ptt);

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== HealthMonitor v2 ===");
  connectWiFi();
  ds18b20.begin();
  initMAX30102();
  collectBatch();
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_SIZE, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHR
  );
  Serial.println("Ready — streaming.");
}

void loop() {
  // Slide buffer
  for (byte i = 25; i < BUFFER_SIZE; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25]  = irBuffer[i];
  }

  // Collect 25 fresh samples
  for (byte i = 75; i < BUFFER_SIZE; i++) {
    while (particleSensor.available() == false)
      particleSensor.check();

    long irValue = particleSensor.getIR();

    // ── PBA beat detection + PTT computation ──────────────────────────────
    if (checkForBeat(irValue)) {
      unsigned long now = millis();
      long delta = now - lastBeat;
      lastBeat   = now;

      beatsPerMinute = 60000.0f / delta;   // delta is already ms here

      if (beatsPerMinute > 20 && beatsPerMinute < 255) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }

      // Roll PTT window: peak2 → peak1, now → peak2
      peakTime1 = peakTime2;
      peakTime2 = now;

      if (peakTime1 > 0) {
        float candidate = (float)(peakTime2 - peakTime1);
        // Accept only physiologically plausible PTT (100–600 ms)
        if (candidate >= 100.0f && candidate <= 600.0f) {
          pttMs = candidate;
        }
      }
    }

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = irValue;
    particleSensor.nextSample();
  }

  // Recalculate SpO2/HR
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_SIZE, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHR
  );

  float ds_c  = readDS18B20();
  float max_c = particleSensor.readTemperature();

  Serial.printf(
    "[%lums] SpO2=%d(%s) HR=%d avg=%d PTT=%.1fms DS=%.2f°C die=%.2f°C\n",
    millis(), spo2, validSPO2?"OK":"??",
    heartRate, beatAvg, pttMs, ds_c, max_c
  );

  if (millis() - lastPushTime >= PUSH_INTERVAL_MS) {
    lastPushTime = millis();
    int32_t bpm = (beatAvg > 0) ? beatAvg : heartRate;
    pushToAPI(ds_c, max_c, bpm, spo2, validHR, validSPO2, pttMs);
  }
}

// ─── Wi-Fi ───────────────────────────────────────────────────────────────────
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
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
    Serial.println("MAX30102 not found!"); while (1);
  }
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  particleSensor.enableDIETEMPRDY();
}

// ─── Initial 100-sample batch ─────────────────────────────────────────────────
void collectBatch() {
  for (byte i = 0; i < BUFFER_SIZE; i++) {
    while (particleSensor.available() == false) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
}

// ─── DS18B20 ─────────────────────────────────────────────────────────────────
float readDS18B20() {
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);
  return (t == DEVICE_DISCONNECTED_C) ? -127.0f : t;
}

// ─── HTTP POST (HTTPS to Render) ──────────────────────────────────────────────
void pushToAPI(float ds_c, float max_c, int32_t hr, int32_t spo2Val,
               bool hrOk, bool spo2Ok, float ptt) {

  if (WiFi.status() != WL_CONNECTED) { connectWiFi(); return; }

  WiFiClientSecure client;
  client.setInsecure();   // Skip cert verification for MVP.
                          // For production: load Render's CA cert instead.

  HTTPClient http;
  String url = String("https://") + API_HOST + API_ENDPOINT;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<320> doc;
  doc["device_id"]       = DEVICE_ID;
  doc["timestamp_ms"]    = (uint32_t)millis();
  doc["bpm"]             = hr;
  doc["bpm_valid"]       = hrOk;
  doc["spo2"]            = spo2Val;
  doc["spo2_valid"]      = spo2Ok;
  doc["temp_body_c"]     = serialized(String(ds_c,  2));
  doc["temp_body_f"]     = serialized(String(ds_c * 9.0 / 5.0 + 32.0, 2));
  doc["temp_die_c"]      = serialized(String(max_c, 2));
  doc["finger_detected"] = (irBuffer[BUFFER_SIZE - 1] >= 50000);

  // Only include ptt_ms when valid
  if (ptt > 0) doc["ptt_ms"] = serialized(String(ptt, 1));

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  Serial.printf("  → POST %s HTTP %d\n", url.c_str(), code);
  http.end();
}
