#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <HTTPClient.h>


const char* ssid = "Galaxy A05 AL";
const char* password = "whywifi1";

// PC IP ADDRESS
const char* serverURL = "http://10.22.68.50:3000/alert";

Adafruit_ADS1115 ads1;   // 0x48
Adafruit_ADS1115 ads2;   // 0x49

// ======================
// REAL-TIME CONFIG
// ======================
#define SAMPLE_RATE     200.0     // Hz
#define WINDOW_SIZE     40        // sliding window
#define SAMPLE_PERIOD_US (1000000.0 / SAMPLE_RATE)

// ADXL335 constantsvg
const float VREF = 4.096;
const float ADC_MAX = 32768.0;
const float ZERO_G = 1.65;
const float SENS = 0.300;
const float G_TO_MS2 = 9.80665;

// ======================
// SLIDING WINDOW BUFFERS (8 SENSORS)
// ======================
float buffer[8][WINDOW_SIZE];
int bufIndex = 0;
bool bufferFilled = false;

unsigned long lastSampleMicros = 0;


bool alertArmed[8] = {true, true, true, true, true, true, true, true};  // One-shot trigger per sensor
unsigned long lastAlertTime[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // Cooldown timer

// ======================
// MIDDLE STATE MACHINE
// ======================

enum MiddleState {
  MIDDLE_CLEAR,
  MIDDLE_APPROACHING,
  MIDDLE_OCCUPIED
};

MiddleState currentMiddleState = MIDDLE_CLEAR;

// Track last sensor that triggered
int lastTriggeredSensor = -1;

// Exit logic variables
enum ExitDirection { NONE, LEFT_EXIT, RIGHT_EXIT };
ExitDirection exitDirection = NONE;
int maxExitIndex = 0;

// Approach tracking
int lastLeftDistance  = 99;
int lastRightDistance = 99;


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

  Serial.println("\n WiFi Connected!");

  Wire.begin(21, 22);

  if (!ads1.begin(0x48)) while (1);
  if (!ads2.begin(0x49)) while (1);

  ads1.setGain(GAIN_ONE);
  ads2.setGain(GAIN_ONE);

  Serial.println(" 8-Sensor Real-Time RMS + Peak Engine Ready");
}

// ======================
// LOOP
// ======================
void loop() {
  if (micros() - lastSampleMicros >= SAMPLE_PERIOD_US) {
    lastSampleMicros += SAMPLE_PERIOD_US;

    // ===========================
    // READ 8 Z-AXIS CHANNELS
    // ===========================
    int16_t raw[8];

    raw[0] = ads1.readADC_SingleEnded(0);  // S1_Z
    raw[1] = ads1.readADC_SingleEnded(1);  // S2_Z
    raw[2] = ads1.readADC_SingleEnded(2);  // S3_Z
    raw[3] = ads1.readADC_SingleEnded(3);  // S4_Z
    raw[4] = ads2.readADC_SingleEnded(0);  // S1_Z
    raw[5] = ads2.readADC_SingleEnded(1);  // S2_Z
    raw[6] = ads2.readADC_SingleEnded(2);  // S3_Z
    raw[7] = ads2.readADC_SingleEnded(3);  // S4_Z

    for (int i = 0; i < 8; i++) {
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

void updateMiddleLogic(int currentSensor) {
  if (lastTriggeredSensor == -1) {
    lastTriggeredSensor = currentSensor;
    return;
  }

  //  NEW RULE: If already approaching and middle sensor hits → OCCUPIED
  if (currentMiddleState == MIDDLE_APPROACHING &&
      (currentSensor == 4 || currentSensor == 5)) {

      currentMiddleState = MIDDLE_OCCUPIED;
      exitDirection = NONE;
      maxExitIndex = 0;
      lastTriggeredSensor = currentSensor;
      return;  //  Stop further processing immediately
  }

  // ======================
  //  1. DETECT MIDDLE ENTRY
  // ======================
  if ((lastTriggeredSensor == 4 && currentSensor == 5) ||
      (lastTriggeredSensor == 5 && currentSensor == 4)) {
    currentMiddleState = MIDDLE_OCCUPIED;
    exitDirection = NONE;
    maxExitIndex = 0;
  }

  // ======================
  //  2. EXIT LOGIC (WITH JITTER)
  // ======================
  if (currentMiddleState == MIDDLE_OCCUPIED) {

    // ---- EXIT RIGHT: 5 → 6 → 7 → 8 ----
    if (exitDirection == NONE && currentSensor == 6) {
      exitDirection = RIGHT_EXIT;
      maxExitIndex = 1;
    }

    if (exitDirection == RIGHT_EXIT) {
      if (currentSensor == 6) maxExitIndex = max(maxExitIndex, 1);
      if (currentSensor == 7) maxExitIndex = max(maxExitIndex, 2);

      if (currentSensor == 8 && maxExitIndex >= 2) {
        currentMiddleState = MIDDLE_CLEAR;
        exitDirection = NONE;
        maxExitIndex = 0;
        lastLeftDistance = 99;
        lastRightDistance = 99;
      }
    }

    // ---- EXIT LEFT: 4 → 3 → 2 → 1 ----
    if (exitDirection == NONE && currentSensor == 3) {
      exitDirection = LEFT_EXIT;
      maxExitIndex = 1;
    }

    if (exitDirection == LEFT_EXIT) {
      if (currentSensor == 3) maxExitIndex = max(maxExitIndex, 1);
      if (currentSensor == 2) maxExitIndex = max(maxExitIndex, 2);

      if (currentSensor == 1 && maxExitIndex >= 2) {
        currentMiddleState = MIDDLE_CLEAR;
        exitDirection = NONE;
        maxExitIndex = 0;
        lastLeftDistance = 99;
        lastRightDistance = 99;
      }
    }

    lastTriggeredSensor = currentSensor;
    return;  //  OCCUPIED always has priority
  }

  // ======================
  //  3. APPROACH DETECTION
  // ======================

  int distanceToMiddleLeft  = (currentSensor <= 4) ? (5 - currentSensor) : 99;
  int distanceToMiddleRight = (currentSensor >= 5) ? (currentSensor - 4) : 99;

  bool approaching = false;

  if (currentSensor <= 4) {  // LEFT SIDE
    if (distanceToMiddleLeft < lastLeftDistance) approaching = true;
    lastLeftDistance = distanceToMiddleLeft;
  }

  if (currentSensor >= 5) {  // RIGHT SIDE
    if (distanceToMiddleRight < lastRightDistance) approaching = true;
    lastRightDistance = distanceToMiddleRight;
  }

  if (approaching) {
    currentMiddleState = MIDDLE_APPROACHING;
  } else {
    currentMiddleState = MIDDLE_CLEAR;
  }

  lastTriggeredSensor = currentSensor;
}


void sendStateToServer() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");

    String stateStr = "CLEAR";

    if (currentMiddleState == MIDDLE_OCCUPIED)
      stateStr = "OCCUPIED";
    else if (currentMiddleState == MIDDLE_APPROACHING)
      stateStr = "APPROACHING";

    String payload = "{";
    payload += "\"state\": \"" + stateStr + "\"";
    payload += "}";

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
      Serial.println(" State sent: " + stateStr);
    } else {
      Serial.println(" Failed to send state");
    }

    http.end();
  }
}


// ===========================
// FAST RMS + PEAK CALCULATION
// ===========================
void computeVibration() {
  static float rmsOut[8] = {0};
  static float peakOut[8] = {0};

  const float ALPHA = 0.2;

  const float PEAK_THRESHOLD  = 2.0;  //  Trigger level
  const float RESET_THRESHOLD = 1.0;  //  Must fall below this to re-arm
  const unsigned long ALERT_COOLDOWN = 1000; //  1 second minimum between alerts

  for (int s = 0; s < 8; s++) {
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

    //  EDGE-TRIGGERED ALERT SYSTEM 

    unsigned long now = millis();

    // SEND ONLY ONCE PER EVENT
    if (peakOut[s] > PEAK_THRESHOLD &&
        alertArmed[s] == true &&
        now - lastAlertTime[s] > ALERT_COOLDOWN) {

      updateMiddleLogic(s + 1);
      sendStateToServer();

      alertArmed[s] = false;      //  Lock until vibration ends
      lastAlertTime[s] = now;    //  Store time
    }

    // RESET ONLY AFTER VIBRATION FALLS BACK DOWN
    if (peakOut[s] < RESET_THRESHOLD) {
      alertArmed[s] = true;      //  Re-arm for next event
    }
  }

  Serial.printf(
    "S1 RMS: %.3f | P: %.3f || "
    "S2 RMS: %.3f | P: %.3f || "
    "S3 RMS: %.3f | P: %.3f || "
    "S4 RMS: %.3f | P: %.3f || "
    "S5 RMS: %.3f | P: %.3f || "
    "S6 RMS: %.3f | P: %.3f || "
    "S7 RMS: %.3f | P: %.3f || "
    "S8 RMS: %.3f | P: %.3f\n",
    rmsOut[0], peakOut[0],
    rmsOut[1], peakOut[1],
    rmsOut[2], peakOut[2],
    rmsOut[3], peakOut[3],
    rmsOut[4], peakOut[4],
    rmsOut[5], peakOut[5],
    rmsOut[6], peakOut[6],
    rmsOut[7], peakOut[7]
  );
}


