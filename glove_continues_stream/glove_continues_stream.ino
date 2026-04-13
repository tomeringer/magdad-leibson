#include <Arduino.h>

// ========================================================
//                  SYSTEM CONFIGURATION
// ========================================================
constexpr int NUM_FLEX = 5; 
int flexPins[NUM_FLEX] = {A0, A2, A1, A3, A7}; 
int servoPins[NUM_FLEX] = {D2, D3, D4, D5, D6}; 
int pwmChannels[NUM_FLEX] = {0, 1, 2, 3, 4}; 

constexpr float R_FIXED = 47000.0f;
constexpr float V_IN = 3.3f;

// ========================================================
//        SERVO CALIBRATION (MANUAL OFFSETS)
// ========================================================
// Adjust these values (positive or negative) to find the true "zero" for each finger.
int servoOffsets[NUM_FLEX] = {80, 85, 120, 90, 80}; 
const int SERVO_STATES[4] = {0, -13, -27, -40};
int currentStates[NUM_FLEX] = {0, 0, 0, 0, 0};
//אצבע משולשת לתקן כיול
//זרת לתקן כיול
// לשנות זוויות בעבור אצבע המורה
// אצבע המורה לתקן כיול


// ========================================================
//        CALIBRATED THRESHOLDS & HYSTERESIS
// ========================================================
// tring manualy
// float THRESHOLDS[NUM_FLEX][3] = {
//   {37002.4f, 45115.9f, 59251.9f},
//   {18688.8f, 26000.0f, 35000.2f},
//   {19430.9f, 26701.9f, 40450.2f},
//   {25493.6f, 30308.9f, 38821.2f},
//   {12044.7f, 14235.7f, 19452.4f}
// };

// // Updated to 2D Array
// float HYSTERESIS[NUM_FLEX][3] = {
//   {1500.0f, 1500.0f, 1500.0f},
//   {1000.0f, 1000.0f, 1000.0f},
//   {1000.0f, 1000.0f, 1000.0f},
//   {857.6f,  857.6f,  857.6f},
//   {700.0f,  700.0f,  700.0f}
// };
float THRESHOLDS[NUM_FLEX][3] = {
  {38406.8f, 49937.3f, 75488.9f},
  {20396.7f, 27556.0f, 44790.9f},
  {22969.2f, 30980.6f, 47308.5f},
  {28157.5f, 32627.5f, 42820.8f},
  {13922.7f, 17090.8f, 26576.7f}
};

float HYSTERESIS[NUM_FLEX][3] = {
  {1878.9f, 9429.3f, 14966.5f},
  {1488.4f, 5537.5f, 11153.6f},
  {3367.6f, 3961.3f, 12128.7f},
  {3310.0f, 935.5f, 9082.1f},
  {927.4f, 2185.0f, 7134.9f}
};
// ========================================================

void setup() {
  Serial.begin(115200);
  while (!Serial);

  for(int i = 0; i < NUM_FLEX; i++) {
    ledcSetup(pwmChannels[i], 50, 14); 
    ledcAttachPin(servoPins[i], pwmChannels[i]); 
  }
  delay(1000); 
}

void setServoAngle(int channel, int angle) {
  int duty = map(angle, 0, 180, 410, 1966);
  ledcWrite(channel, duty); 
}

int getQuantizedState(float r, int fingerIndex) {
  int state = currentStates[fingerIndex];

  // Look UP a state, using the hysteresis for the current gap
  while (state < 3 && r > (THRESHOLDS[fingerIndex][state] + HYSTERESIS[fingerIndex][state])) {
      state++;
  }
  // Look DOWN a state, using the hysteresis for the gap below
  while (state > 0 && r < (THRESHOLDS[fingerIndex][state - 1] - HYSTERESIS[fingerIndex][state - 1])) {
      state--;
  }
  return state;
}

void loop() {
  for (int i = 0; i < NUM_FLEX; i++) {
    long sum = 0;
    for (int k = 0; k < 5; k++) {
      sum += analogRead(flexPins[i]);
    }
    
    float v = (sum / 5.0f) * (V_IN / 4095.0f);
    float r = 0.0f;
    
    // Hardware safety to prevent inf
    if (v > 0.01f) {
      if (v >= V_IN - 0.01f) {
        r = 1000000.0f; 
      } else {
        r = R_FIXED * (v / (V_IN - v));
      }
    }

    int newState = getQuantizedState(r, i);
    currentStates[i] = newState;
    
    // --- APPLY OFFSET AND CONSTRAIN ---
    int baseAngle = SERVO_STATES[currentStates[i]];
    int calibratedAngle = baseAngle + servoOffsets[i];
    
    // Prevent the angle from exceeding physical limits (0 to 180 degrees)
    calibratedAngle = constrain(calibratedAngle, 0, 180);
    
    setServoAngle(pwmChannels[i], calibratedAngle);
    
    // Stream Resistance AND State
    Serial.print(r, 1);
    Serial.print(",");
    Serial.print(currentStates[i]);
    
    if (i < NUM_FLEX - 1) {
      Serial.print(",");
    }
  }
  
  Serial.println(); 
  delay(20); 
}