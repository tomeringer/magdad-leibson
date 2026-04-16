#include <Arduino.h>

// ========================================================
//                  SYSTEM CONFIGURATION
// ========================================================
constexpr int NUM_FLEX = 5; 
// REORDERED PINS: f0(A3), f1(A0), f2(A2), f3(A7), f4(A1)
int flexPins[NUM_FLEX] = {A3, A0, A2, A7, A1}; 
// Reordered servos to stay paired with their respective fingers:
int servoPins[NUM_FLEX] = {D5, D2, D3, D6, D4}; 
int pwmChannels[NUM_FLEX] = {0, 1, 2, 3, 4}; 

constexpr float R_FIXED = 47000.0f;
constexpr float V_IN = 3.3f;

// ========================================================
//        SERVO CALIBRATION (MANUAL OFFSETS)
// ========================================================
// Reordered offsets to match the new finger order
int servoOffsets[NUM_FLEX] = {90, 80, 85, 80, 120}; 
const int SERVO_STATES[4] = {0, -13, -27, -40};
int currentStates[NUM_FLEX] = {0, 0, 0, 0, 0};

//אצבע משולשת לתקן כיול
//זרת לתקן כיול
// לשנות זוויות בעבור אצבע המורה
// אצבע המורה לתקן כיול

// ========================================================
//        CALIBRATED THRESHOLDS & HYSTERESIS
// ========================================================
// Statistically calculated Thresholds (Reordered to match new pins)
// float THRESHOLDS[NUM_FLEX][3] = {
//   {28157.5f, 32627.5f, 42820.8f}, // f0 (A3)
//   {38406.8f, 49937.3f, 75488.9f}, // f1 (A0)
//   {20396.7f, 27556.0f, 44790.9f}, // f2 (A2)
//   {13922.7f, 17090.8f, 26576.7f}, // f3 (A7)
//   {22969.2f, 30980.6f, 47308.5f}  // f4 (A1)
// };

// // Expanded Hysteresis Matrix (Reordered to match new pins)
// float HYSTERESIS[NUM_FLEX][3] = {
//   {3310.0f,  935.5f,  9082.1f},   // f0 (A3)
//   {1878.9f, 9429.3f, 14966.5f},   // f1 (A0)
//   {1488.4f, 5537.5f, 11153.6f},   // f2 (A2)
//   { 927.4f, 2185.0f,  7134.9f},   // f3 (A7)
//   {3367.6f, 3961.3f, 12128.7f}    // f4 (A1)
// };

float THRESHOLDS[NUM_FLEX][3] = {
  {26485.3f, 32603.6f, 40369.7f}, // f0 (A3)       
  {37688.6f, 53299.8f, 77914.6f}, // f1 (A0)       
  {20447.2f, 30992.2f, 46028.6f}, // f2 (A2)       
  {15006.7f, 20655.7f, 29523.2f}, // f3 (A7)       
  {23089.4f, 33341.3f, 46980.6f}  // f4 (A1)       
};

float HYSTERESIS[NUM_FLEX][3] = {
  {3169.5f, 1543.5f, 4551.7f}, // f0 (A3)
  {6214.7f, 7303.7f, 14451.8f}, // f1 (A0)
  {3850.7f, 5570.2f, 7433.3f}, // f2 (A2)
  {2289.7f, 2777.2f, 5202.9f}, // f3 (A7)
  {4388.2f, 4579.6f, 7535.8f}  // f4 (A1)
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