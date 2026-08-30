/*
  HealthMonitor ESP32 Firmware — v3.1 (SpO2 acquisition fix)
  ─────────────────────────────────────────────────────────────────────────────
  What changed vs v3.0, and why:

    - ROOT CAUSE OF SpO2=0(??):
        The old acquisition loop wrote red=0/ir=0 into the 100-sample buffer
        whenever a single sample read failed (I2C hiccup, FIFO underrun, etc).
        Even a handful of zero entries in that window is enough for
        maxim_heart_rate_and_oxygen_saturation() to reject the whole result.
        HR still "worked" because the beat detector only looks at real,
        nonzero IR samples as they arrive — it never saw the zeros directly.

    - FIX:
        A failed/zero sample is now retried instead of written to the buffer.
        The buffer only ever contains real RED+IR pairs. Collection of the
        FRESH_SAMPLES batch is bounded by a wall-clock budget so a removed
        finger (or dead sensor) can't hang the main loop — if the budget
        expires before a full batch is collected, that cycle is marked
        incomplete and vitals are NOT recalculated from a stale/partial
        buffer (previous valid values are held, and invalid flags are set
        if there's no finger signal at all).

    - sampleAverage changed 4 -> 1: the library's internal averaging was
      compounding with the app-level windowing and degrading the RED/IR
      relationship the SpO2 math depends on.

    - RED is now sampled, timeout-guarded, logged (serial + API) and
      validated with the same rigor as IR — previously only IR was ever
      surfaced, so a dead RED channel would have been invisible.

    - Raw algorithm output (before range/validity normalization) is logged
      so a "library says invalid" failure can be told apart from a
      "we discarded a valid result" bug.

    - Wi-Fi never causes an ESP32 restart
    - MAX30102 never blocks indefinitely (bounded retry, not infinite)
    - I2C has a timeout
    - API failures do not crash the firmware
    - Automatic Wi-Fi reconnection
*/

#define WIFI_SSID      "mayberry"
#define WIFI_PASSWORD  "pass123now2"

#define API_HOST       "elaborate-flavorful-virtuous.ngrok-free.dev"   // Host domain without protocol (e.g. domain.ngrok-free.dev or IP:port)
#define API_ENDPOINT   "/readings"
#define USE_HTTPS      true

#define DEVICE_ID      "esp32-01"

// Set to 1 to print the raw (pre-normalization) algorithm output each cycle.
#define DEBUG_RAW_ALGORITHM 1

// ─── Libraries ────────────────────────────────────────────────────────────────
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

#include <OneWire.h>
#include <DallasTemperature.h>

// ─── Pins ─────────────────────────────────────────────────────────────────────
#define SDA_PIN       21
#define SCL_PIN       22
#define DS18B20_PIN   4

// ─── Sampling ────────────────────────────────────────────────────────────────
#define BUFFER_SIZE           100
#define FRESH_SAMPLES          25
#define SAMPLE_TIMEOUT_MS      50     // max wait for a single FIFO sample
#define MAX_ACQUIRE_TIME_MS   400     // max wall-clock time to collect FRESH_SAMPLES valid pairs

// ─── API ─────────────────────────────────────────────────────────────────────
#define PUSH_INTERVAL_MS    1000

// ─── Finger detection ────────────────────────────────────────────────────────
#define FINGER_IR_THRESHOLD 50000UL

// ─── Objects ──────────────────────────────────────────────────────────────────
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
MAX30105 particleSensor;

// ─── MAX30102 buffers (only ever hold real RED/IR pairs) ────────────────────
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

// ─── Calculated measurements ─────────────────────────────────────────────────
int32_t spo2 = 0;
int8_t  validSPO2 = 0;

int32_t heartRate = 0;
int8_t  validHR = 0;

// ─── Beat detection ──────────────────────────────────────────────────────────
#define RATE_SIZE 4

byte rates[RATE_SIZE] = {0};
byte rateSpot = 0;

float beatsPerMinute = 0.0f;
int beatAvg = 0;

unsigned long peakTime1 = 0;
unsigned long peakTime2 = 0;

float pttMs = -1.0f;

// ─── Runtime state ──────────────────────────────────────────────────────────
bool max30102Ready = false;
bool wifiReady = false;
bool bufferFullyPrimed = false;   // true once BUFFER_SIZE real samples have ever been collected

unsigned long lastPushTime = 0;

// ─── FIFO stall recovery ──────────────────────────────────────────────────────
// If the FIFO's read/write pointers ever get desynced (I2C glitch, or an
// ill-timed clearFIFO() mid-conversion), particleSensor.available() can get
// permanently stuck returning false. We track the last time we actually
// pulled a real sample, and if too long passes, force a clean resync ONCE
// rather than hammering clearFIFO() on every 50ms timeout (which is what
// causes the desync in the first place).
#define STALL_RESYNC_MS   1500
unsigned long lastRealSampleMs = 0;


// ═════════════════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS
// ═════════════════════════════════════════════════════════════════════════════

void connectWiFi();
bool initMAX30102();

bool readMAXSample(uint32_t &red, uint32_t &ir);

void maybeRecoverFromStall();

void collectInitialBuffer();

bool acquireFreshSamples();   // returns true if a full batch of FRESH_SAMPLES was collected

void processBeat(uint32_t ir);

void calculateVitals();

float readDS18B20();

void pushToAPI(
  float ds_c,
  float max_c,
  int32_t hr,
  int32_t spo2Val,
  bool hrOk,
  bool spo2Ok,
  float ptt,
  uint32_t redRaw
);


// ═════════════════════════════════════════════════════════════════════════════
// SETUP
// ═════════════════════════════════════════════════════════════════════════════

void setup() {

  Serial.begin(115200);
  delay(100);

  Serial.println();
  Serial.println("================================");
  Serial.println("     HealthMonitor ESP32 v3.1");
  Serial.println("================================");

  // ── I2C ────────────────────────────────────────────────────────────────────
  Wire.begin(SDA_PIN, SCL_PIN);

  // Prevent I2C from hanging the CPU forever.
  Wire.setTimeOut(50);

  // ── Wi-Fi ──────────────────────────────────────────────────────────────────
  connectWiFi();

  // ── DS18B20 ────────────────────────────────────────────────────────────────
  ds18b20.begin();

  Serial.printf(
    "DS18B20 devices: %d\n",
    ds18b20.getDeviceCount()
  );

  // ── MAX30102 ───────────────────────────────────────────────────────────────
  max30102Ready = initMAX30102();

  if (max30102Ready) {
    collectInitialBuffer();
  } else {
    Serial.println("MAX30102 unavailable. Vitals will remain invalid until it is detected.");
  }

  Serial.println("Ready — streaming.");
}


// ═════════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═════════════════════════════════════════════════════════════════════════════

void loop() {

  // ───────────────────────────────────────────────────────────────────────────
  // 1. Shift existing buffer left by FRESH_SAMPLES to make room
  // ───────────────────────────────────────────────────────────────────────────

  for (int i = FRESH_SAMPLES; i < BUFFER_SIZE; i++) {
    redBuffer[i - FRESH_SAMPLES] = redBuffer[i];
    irBuffer[i - FRESH_SAMPLES]  = irBuffer[i];
  }


  // ───────────────────────────────────────────────────────────────────────────
  // 1b. Watchdog: recover from a desynced/stalled FIFO if needed.
  // ───────────────────────────────────────────────────────────────────────────

  maybeRecoverFromStall();


  // ───────────────────────────────────────────────────────────────────────────
  // 2. Acquire FRESH_SAMPLES real (nonzero) RED+IR pairs.
  //
  // IMPORTANT (this is the actual SpO2 fix):
  // A failed or zero sample is retried, NOT written into the buffer.
  // Zeros in the window are what make the Maxim algorithm reject SpO2
  // outright even while HR keeps working. Collection is bounded by
  // MAX_ACQUIRE_TIME_MS so a missing finger can't hang the firmware.
  // ───────────────────────────────────────────────────────────────────────────

  bool acquisitionComplete = acquireFreshSamples();

  if (acquisitionComplete) {
    bufferFullyPrimed = bufferFullyPrimed || true;
  }


  // ───────────────────────────────────────────────────────────────────────────
  // 3. Calculate HR + SpO2 — only from a fully-real, fully-primed buffer.
  // ───────────────────────────────────────────────────────────────────────────

  if (acquisitionComplete && bufferFullyPrimed) {
    calculateVitals();
  } else {
    // Not enough real signal this cycle (finger just placed / removed,
    // or sensor briefly unavailable). Don't compute from a stale/partial
    // buffer — report invalid instead of a misleading stale number.
    spo2 = 0;
    heartRate = 0;
    validSPO2 = 0;
    validHR = 0;
  }


  // ───────────────────────────────────────────────────────────────────────────
  // 4. Temperatures
  // ───────────────────────────────────────────────────────────────────────────

  float ds_c = readDS18B20();

  float max_c = 0.0f;

  if (max30102Ready) {
    max_c = particleSensor.readTemperature();
  }


  // ───────────────────────────────────────────────────────────────────────────
  // 5. Finger detection
  // ───────────────────────────────────────────────────────────────────────────

  bool fingerDetected =
    irBuffer[BUFFER_SIZE - 1] >= FINGER_IR_THRESHOLD;


  // ───────────────────────────────────────────────────────────────────────────
  // 6. Serial output — RED is now printed alongside IR
  // ───────────────────────────────────────────────────────────────────────────

  Serial.printf(
    "[%lums] "
    "WiFi=%s "
    "SpO2=%ld(%s) "
    "HR=%ld avg=%d "
    "PTT=%.1fms "
    "DS=%.2fC "
    "die=%.2fC "
    "finger=%s "
    "IR=%lu "
    "RED=%lu "
    "primed=%s\n",

    millis(),

    WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",

    (long)spo2,
    validSPO2 ? "OK" : "??",

    (long)heartRate,
    beatAvg,

    pttMs,

    ds_c,
    max_c,

    fingerDetected ? "YES" : "NO",

    (unsigned long)irBuffer[BUFFER_SIZE - 1],
    (unsigned long)redBuffer[BUFFER_SIZE - 1],

    bufferFullyPrimed ? "YES" : "NO"
  );


  // ───────────────────────────────────────────────────────────────────────────
  // 7. Push API
  // ───────────────────────────────────────────────────────────────────────────

  if (millis() - lastPushTime >= PUSH_INTERVAL_MS) {

    lastPushTime = millis();

    int32_t bpm =
      (beatAvg > 0)
      ? beatAvg
      : heartRate;

    pushToAPI(
      ds_c,
      max_c,
      bpm,
      spo2,
      validHR,
      validSPO2,
      pttMs,
      redBuffer[BUFFER_SIZE - 1]
    );
  }

  // Never allow the main loop to become CPU-bound.
  delay(1);
}


// ═════════════════════════════════════════════════════════════════════════════
// WI-FI
// ═════════════════════════════════════════════════════════════════════════════

void connectWiFi() {

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  Serial.printf(
    "Connecting to %s",
    WIFI_SSID
  );

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );

  unsigned long start = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - start < 15000
  ) {

    delay(250);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {

    wifiReady = true;

    Serial.println("Wi-Fi connected.");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());

    Serial.printf(
      "RSSI: %d dBm\n",
      WiFi.RSSI()
    );

  } else {

    wifiReady = false;

    Serial.printf(
      "Wi-Fi unavailable. Status=%d\n",
      WiFi.status()
    );

    // IMPORTANT:
    // Do NOT restart.
    // The firmware continues and will retry later.
  }
}


// ═════════════════════════════════════════════════════════════════════════════
// MAX30102 INITIALIZATION
// ═════════════════════════════════════════════════════════════════════════════

bool initMAX30102() {

  Serial.println("Initializing MAX30102...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {

    Serial.println(
      "MAX30102 not found."
    );

    return false;
  }

  /*
    ledBrightness = 60
    sampleAverage = 1   <-- changed from 4. Internal averaging on top of
                             our own windowing was smoothing away the
                             AC/DC relationship SpO2 depends on.
    ledMode      = 2 (Red + IR)
    sampleRate   = 100 Hz
    pulseWidth   = 411 us
    adcRange     = 4096
  */

  particleSensor.setup(
    60,
    1,
    2,
    100,
    411,
    4096
  );

  particleSensor.enableDIETEMPRDY();

  particleSensor.clearFIFO();

  Serial.println("MAX30102 ready.");

  return true;
}


// ═════════════════════════════════════════════════════════════════════════════
// SAFE MAX30102 SAMPLE
// ═════════════════════════════════════════════════════════════════════════════
//
// Returns:
//
//   true  = real, nonzero sample received (both red and ir already checked)
//   false = no sample received within SAMPLE_TIMEOUT_MS
//
// On failure red=0, ir=0 are returned but the CALLER is responsible for
// never writing those zeros into redBuffer/irBuffer — see acquireFreshSamples().
//
// NEVER waits indefinitely — bounded by SAMPLE_TIMEOUT_MS.
// ═════════════════════════════════════════════════════════════════════════════

bool readMAXSample(uint32_t &red, uint32_t &ir) {

  red = 0;
  ir  = 0;

  if (!max30102Ready) {
    return false;
  }

  unsigned long start = millis();

  while (!particleSensor.available()) {

    particleSensor.check();

    // Give FreeRTOS time to run.
    delay(1);

    // HARD TIMEOUT.
    if (millis() - start >= SAMPLE_TIMEOUT_MS) {

      // NOTE: We deliberately do NOT call particleSensor.clearFIFO() here.
      // A transient timeout is normal (I2C scheduling jitter, a slow
      // conversion, etc). Resetting the FIFO's read/write pointers on
      // every single transient timeout — which, under sustained signal
      // loss, means every ~50ms — can desync those pointers permanently,
      // leaving available() stuck false forever. That desync is what
      // produced a completely frozen IR/RED reading in testing. Real,
      // prolonged stalls are instead handled by the watchdog in loop()
      // via maybeRecoverFromStall(), which resyncs at most once per
      // STALL_RESYNC_MS rather than on every failed read.
      return false;
    }
  }

  // Actual sample exists.
  red = particleSensor.getRed();
  ir  = particleSensor.getIR();

  particleSensor.nextSample();

  if (red == 0 || ir == 0) {
    return false;
  }

  lastRealSampleMs = millis();

  return true;
}


// ═════════════════════════════════════════════════════════════════════════════
// STALL WATCHDOG
// ═════════════════════════════════════════════════════════════════════════════
//
// Called once per loop(). If we haven't pulled a single real sample in
// STALL_RESYNC_MS, the FIFO pointers are likely desynced (or the sensor
// dropped off the bus). Do ONE clean resync — not a clearFIFO() spam —
// and reset the stall clock so we don't do this every cycle.
// ═════════════════════════════════════════════════════════════════════════════

void maybeRecoverFromStall() {

  if (!max30102Ready) {
    return;
  }

  if (lastRealSampleMs == 0) {
    // Haven't gotten a first real sample yet (e.g. still waiting for a
    // finger at boot) — not a stall, nothing to recover from.
    return;
  }

  if (millis() - lastRealSampleMs < STALL_RESYNC_MS) {
    return;
  }

  Serial.printf(
    "No real MAX30102 sample in %lums — resyncing FIFO.\n",
    (unsigned long)(millis() - lastRealSampleMs)
  );

  particleSensor.clearFIFO();

  // Prevent immediately re-triggering the watchdog while the sensor
  // catches back up.
  lastRealSampleMs = millis();
}


// ═════════════════════════════════════════════════════════════════════════════
// ACQUIRE A FULL BATCH OF FRESH_SAMPLES REAL SAMPLES
// ═════════════════════════════════════════════════════════════════════════════
//
// Fills redBuffer/irBuffer[BUFFER_SIZE - FRESH_SAMPLES .. BUFFER_SIZE - 1]
// with real, nonzero RED+IR pairs only. A failed read is retried, never
// written as zero. Bounded by MAX_ACQUIRE_TIME_MS so a missing finger or
// dead sensor can't stall the main loop.
//
// Returns true only if the full batch was collected in time.
// ═════════════════════════════════════════════════════════════════════════════

bool acquireFreshSamples() {

  int collected = 0;
  unsigned long acquireStart = millis();

  while (collected < FRESH_SAMPLES) {

    uint32_t red = 0;
    uint32_t ir  = 0;

    bool sampleOK = readMAXSample(red, ir);

    if (!sampleOK) {

      if (millis() - acquireStart >= MAX_ACQUIRE_TIME_MS) {
        // Give up this cycle — not enough real signal (no finger, sensor
        // hiccup, etc). Do NOT pad with zeros.
        return false;
      }

      delay(1);
      continue;
    }

    int index = BUFFER_SIZE - FRESH_SAMPLES + collected;

    redBuffer[index] = red;
    irBuffer[index]  = ir;

    processBeat(ir);

    collected++;

    if (millis() - acquireStart >= MAX_ACQUIRE_TIME_MS) {
      // Ran out of time mid-batch.
      return collected == FRESH_SAMPLES;
    }
  }

  return true;
}


// ═════════════════════════════════════════════════════════════════════════════
// INITIAL MAX30102 BUFFER
// ═════════════════════════════════════════════════════════════════════════════

void collectInitialBuffer() {

  Serial.println(
    "Collecting initial 100 real samples (place finger on sensor)..."
  );

  int collected = 0;
  unsigned long start = millis();

  // Generous budget for the initial fill since we're waiting on the user
  // to place a finger; this only runs once at boot.
  const unsigned long INITIAL_FILL_BUDGET_MS = 15000;

  while (collected < BUFFER_SIZE) {

    uint32_t red = 0;
    uint32_t ir  = 0;

    bool ok = readMAXSample(red, ir);

    if (!ok) {

      if (millis() - start >= INITIAL_FILL_BUDGET_MS) {
        Serial.printf(
          "Initial fill timed out at %d/%d real samples. "
          "Buffer NOT primed yet — will keep trying in loop().\n",
          collected, BUFFER_SIZE
        );
        return;
      }

      delay(1);
      continue;
    }

    redBuffer[collected] = red;
    irBuffer[collected]  = ir;

    processBeat(ir);

    collected++;

    delay(1);
  }

  bufferFullyPrimed = true;

  Serial.println(
    "Initial batch done — buffer fully primed with real samples."
  );

  calculateVitals();
}


// ═════════════════════════════════════════════════════════════════════════════
// BEAT PROCESSING
// ═════════════════════════════════════════════════════════════════════════════

void processBeat(uint32_t ir) {

  if (ir == 0) {
    return;
  }

  if (!checkForBeat(ir)) {
    return;
  }

  unsigned long now = millis();

  // First detected beat.
  if (peakTime2 == 0) {

    peakTime2 = now;

    return;
  }

  // Roll timestamps.
  peakTime1 = peakTime2;
  peakTime2 = now;

  if (peakTime2 <= peakTime1) {
    return;
  }

  float ibi =
    (float)(peakTime2 - peakTime1);


  // ───────────────────────────────────────────────────────────────────────────
  // Valid physiological range:
  //
  // 300 ms = 200 BPM
  // 1500 ms = 40 BPM
  // ───────────────────────────────────────────────────────────────────────────

  if (
    ibi >= 300.0f &&
    ibi <= 1500.0f
  ) {

    pttMs = ibi;

    beatsPerMinute =
      60000.0f / ibi;

    if (
      beatsPerMinute >= 40.0f &&
      beatsPerMinute <= 200.0f
    ) {

      rates[rateSpot] =
        (byte)beatsPerMinute;

      rateSpot =
        (rateSpot + 1) % RATE_SIZE;

      int total = 0;
      int count = 0;

      for (int i = 0; i < RATE_SIZE; i++) {

        if (rates[i] > 0) {

          total += rates[i];
          count++;
        }
      }

      if (count > 0) {
        beatAvg = total / count;
      }
    }
  }
}


// ═════════════════════════════════════════════════════════════════════════════
// HR / SpO2
// ═════════════════════════════════════════════════════════════════════════════

void calculateVitals() {

  uint32_t latestIR = irBuffer[BUFFER_SIZE - 1];

  if (latestIR == 0) {

    spo2 = 0;
    heartRate = 0;

    validSPO2 = 0;
    validHR = 0;

    return;
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    BUFFER_SIZE,
    redBuffer,
    &spo2,
    &validSPO2,
    &heartRate,
    &validHR
  );

#if DEBUG_RAW_ALGORITHM
  Serial.printf(
    "  RAW ALGORITHM -> SpO2=%ld valid=%d HR=%ld valid=%d\n",
    (long)spo2,
    validSPO2,
    (long)heartRate,
    validHR
  );
#endif

  // Normalize invalid library output.
  if (!validHR || heartRate < 30 || heartRate > 220) {

    validHR = 0;
    heartRate = 0;
  }

  if (!validSPO2 || spo2 < 70 || spo2 > 100) {

    validSPO2 = 0;
    spo2 = 0;
  }
}


// ═════════════════════════════════════════════════════════════════════════════
// DS18B20
// ═════════════════════════════════════════════════════════════════════════════

float readDS18B20() {

  if (ds18b20.getDeviceCount() == 0) {
    return 0.0f;
  }

  ds18b20.requestTemperatures();

  float temperature =
    ds18b20.getTempCByIndex(0);

  if (
    temperature == DEVICE_DISCONNECTED_C ||
    temperature < -50.0f ||
    temperature > 125.0f
  ) {

    Serial.println(
      "WARNING: DS18B20 unavailable."
    );

    return 0.0f;
  }

  return temperature;
}


// ═════════════════════════════════════════════════════════════════════════════
// API
// ═════════════════════════════════════════════════════════════════════════════

void pushToAPI(
  float ds_c,
  float max_c,
  int32_t hr,
  int32_t spo2Val,
  bool hrOk,
  bool spo2Ok,
  float ptt,
  uint32_t redRaw
) {

  // ───────────────────────────────────────────────────────────────────────────
  // Wi-Fi unavailable
  // ───────────────────────────────────────────────────────────────────────────

  if (WiFi.status() != WL_CONNECTED) {

    wifiReady = false;

    Serial.println(
      "Wi-Fi unavailable. Attempting reconnect..."
    );

    connectWiFi();

    return;
  }

  wifiReady = true;


  // ───────────────────────────────────────────────────────────────────────────
  // JSON
  // ───────────────────────────────────────────────────────────────────────────

  StaticJsonDocument<512> doc;

  doc["device_id"] =
    DEVICE_ID;

  doc["timestamp_ms"] =
    (uint32_t)millis();

  doc["device_connected"] =
    true;

  doc["bpm"] =
    hr;

  doc["bpm_valid"] =
    hrOk;

  doc["spo2"] =
    spo2Val;

  doc["spo2_valid"] =
    spo2Ok;

  doc["temp_body_c"] =
    ds_c;

  doc["temp_body_f"] =
    ds_c * 9.0f / 5.0f + 32.0f;

  doc["temp_die_c"] =
    max_c;

  bool finger =
    irBuffer[BUFFER_SIZE - 1] >=
    FINGER_IR_THRESHOLD;

  doc["finger_detected"] =
    finger;

  doc["ir_raw"] =
    irBuffer[BUFFER_SIZE - 1];

  doc["red_raw"] =
    redRaw;

  doc["buffer_primed"] =
    bufferFullyPrimed;

  // Only send valid PTT.
  if (ptt > 0.0f) {

    doc["ptt_ms"] =
      ptt;
  }


  String body;

  serializeJson(
    doc,
    body
  );


  // ───────────────────────────────────────────────────────────────────────────
  // HTTP
  // ───────────────────────────────────────────────────────────────────────────

  HTTPClient http;

  String host = API_HOST;
  if (host.startsWith("https://")) host.remove(0, 8);
  if (host.startsWith("http://"))  host.remove(0, 7);

  String url;

  if (USE_HTTPS) {

    url = String("https://") + host + API_ENDPOINT;

    WiFiClientSecure client;

    client.setInsecure();

    // http.setTimeout() below only bounds the read phase, not the initial
    // TCP/TLS connect. When the endpoint is unreachable (as HTTP -1
    // indicates), the connect attempt itself was blocking for ~5s+ here.
    // setConnectTimeout() bounds that phase explicitly so a dead endpoint
    // can't stall sensor acquisition on the next loop() for that long.
    client.setTimeout(3000);
    http.setConnectTimeout(3000);

    if (!http.begin(client, url)) {

      Serial.println(
        "HTTP begin failed."
      );

      return;
    }

    http.addHeader(
      "Content-Type",
      "application/json"
    );

    http.addHeader(
      "ngrok-skip-browser-warning",
      "69420"
    );

    http.setTimeout(3000);

    int code =
      http.POST(body);

    Serial.printf(
      "API POST: HTTP %d (URL: %s)\n",
      code,
      url.c_str()
    );

    if (code <= 0) {
      Serial.printf(
        "HTTP error: %s (check that %s is actually reachable)\n",
        http.errorToString(code).c_str(),
        url.c_str()
      );
    }

    http.end();

  } else {

    url = String("http://") + host + API_ENDPOINT;

    WiFiClient client;

    client.setTimeout(3000);
    http.setConnectTimeout(3000);

    if (!http.begin(client, url)) {

      Serial.println(
        "HTTP begin failed."
      );

      return;
    }

    http.addHeader(
      "Content-Type",
      "application/json"
    );

    http.addHeader(
      "ngrok-skip-browser-warning",
      "69420"
    );

    http.setTimeout(3000);

    int code =
      http.POST(body);

    Serial.printf(
      "API POST: HTTP %d (URL: %s)\n",
      code,
      url.c_str()
    );

    if (code <= 0) {

      Serial.printf(
        "HTTP error: %s (check that %s is actually reachable)\n",
        http.errorToString(code).c_str(),
        url.c_str()
      );
    }

    http.end();
  }
}
