#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <HTTPClient.h>


const char* ssid = "Galaxy A05 AL";
const char* password = "whywifi1";

// ✅ CHANGE THIS TO YOUR PC IP ADDRESS
const char* serverURL = "http://10.122.70.50:3000/alert";

Adafruit_ADS1115 ads1;   // 0x48
Adafruit_ADS1115 ads2;   // 0x49

// ======================
// REAL-TIME CONFIG
// ======================
#define SAMPLE_RATE     200.0     // Hz
#define WINDOW_SIZE     40        // sliding window
#define SAMPLE_PERIOD_US (1000000.0 / SAMPLE_RATE)

// ADXL335 constants
const float VREF = 4.096;
const float ADC_MAX = 32768.0;
const float ZERO_G = 1.65;
const float SENS = 0.300;
const float G_TO_MS2 = 9.80665;

// ======================
// SLIDING WINDOW BUFFERS (4 SENSORS)
// ======================
float buffer[4][WINDOW_SIZE];
int bufIndex = 0;
bool bufferFilled = false;

unsigned long lastSampleMicros = 0;


bool alertArmed[4] = {true, true, true, true};  // ✅ One-shot trigger per sensor
unsigned long lastAlertTime[4] = {0, 0, 0, 0};  // ✅ Cooldown timer

// ======================
// SETUP
// ======================
void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ WiFi Connected!");

  Wire.begin(21, 22);

  if (!ads1.begin(0x48)) while (1);
  if (!ads2.begin(0x49)) while (1);

  ads1.setGain(GAIN_ONE);
  ads2.setGain(GAIN_ONE);

  Serial.println("✅ 4-Sensor Real-Time RMS + Peak Engine Ready");
}

// ======================
// LOOP
// ======================
void loop() {
  if (micros() - lastSampleMicros >= SAMPLE_PERIOD_US) {
    lastSampleMicros += SAMPLE_PERIOD_US;

    // ===========================
    // READ 4 Z-AXIS CHANNELS
    // ===========================
    int16_t raw[4];

    raw[0] = ads1.readADC_SingleEnded(1);  // S1_Z
    raw[1] = ads1.readADC_SingleEnded(3);  // S2_Z
    raw[2] = ads2.readADC_SingleEnded(1);  // S3_Z
    raw[3] = ads2.readADC_SingleEnded(3);  // S4_Z

    for (int i = 0; i < 4; i++) {
      float voltage = raw[i] * VREF / ADC_MAX;
      float accel_g = (voltage - ZERO_G) / SENS;
      float accel = accel_g * G_TO_MS2;

      buffer[i][bufIndex] = accel;
    }

    bufIndex++;
    if (bufIndex >= WINDOW_SIZE) {
      bufIndex = 0;
      bufferFilled = true;
    }

    if (bufferFilled) {
      computeVibration();
    }
  }
}


void sendAlertToServer(int sensorID, float peakVal) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");

    String payload = "{";
    payload += "\"sensor\": " + String(sensorID);
    payload += ", \"peak\": " + String(peakVal, 3);
    payload += "}";

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
      Serial.println("✅ Alert sent to server");
    } else {
      Serial.println("❌ Failed to send alert");
    }

    http.end();
  }
}

// ===========================
// FAST RMS + PEAK CALCULATION
// ===========================
void computeVibration() {
  static float rmsOut[4] = {0};
  static float peakOut[4] = {0};

  const float ALPHA = 0.2;

  const float PEAK_THRESHOLD  = 2.0;  // ✅ Trigger level
  const float RESET_THRESHOLD = 1.0;  // ✅ Must fall below this to re-arm
  const unsigned long ALERT_COOLDOWN = 1000; // ✅ 1 second minimum between alerts

  for (int s = 0; s < 4; s++) {
    float sumSq = 0;
    float maxVal = 0;

    float mean = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) mean += buffer[s][i];
    mean /= WINDOW_SIZE;

    for (int i = 0; i < WINDOW_SIZE; i++) {
      float v = buffer[s][i] - mean;
      sumSq += v * v;
      if (fabs(v) > maxVal) maxVal = fabs(v);
    }

    float rmsRaw  = sqrt(sumSq / WINDOW_SIZE);
    float peakRaw = maxVal;

    rmsOut[s]  = (ALPHA * rmsRaw)  + ((1 - ALPHA) * rmsOut[s]);
    peakOut[s] = (ALPHA * peakRaw) + ((1 - ALPHA) * peakOut[s]);

    // ✅ ✅ ✅ PROPER EDGE-TRIGGERED ALERT SYSTEM ✅ ✅ ✅

    unsigned long now = millis();

    // ✅ SEND ONLY ONCE PER EVENT
    if (peakOut[s] > PEAK_THRESHOLD &&
        alertArmed[s] == true &&
        now - lastAlertTime[s] > ALERT_COOLDOWN) {

      sendAlertToServer(s + 1, peakOut[s]);

      alertArmed[s] = false;      // 🔒 Lock until vibration ends
      lastAlertTime[s] = now;    // 🕒 Store time
    }

    // ✅ RESET ONLY AFTER VIBRATION FALLS BACK DOWN
    if (peakOut[s] < RESET_THRESHOLD) {
      alertArmed[s] = true;      // 🔓 Re-arm for next event
    }
  }

  Serial.printf(
    "S1 RMS: %.3f | P: %.3f || "
    "S2 RMS: %.3f | P: %.3f || "
    "S3 RMS: %.3f | P: %.3f || "
    "S4 RMS: %.3f | P: %.3f\n",
    rmsOut[0], peakOut[0],
    rmsOut[1], peakOut[1],
    rmsOut[2], peakOut[2],
    rmsOut[3], peakOut[3]
  );
}




