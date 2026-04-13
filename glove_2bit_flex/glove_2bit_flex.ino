#include <Wire.h>
#include <ICM_20948.h>

// ========================================================
//          BYTE → BINARY PRINT (For Debugging)
// ========================================================
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

// ========================================================
//                  IMU SETUP
// ========================================================
ICM_20948_I2C imu;
#define AD0_VAL 0

constexpr float ACC_MG_TO_G = 0.001f;
constexpr float ALPHA_RP  = 0.98f;
constexpr float ALPHA_YAW = 0.98f;
constexpr float MAX_DT = 0.1f;

constexpr float ROLL_TH  = 45.0f;
constexpr float PITCH_TH = 30.0f;

// Gyro bias
constexpr float GYRO_BIAS_DPS_X = -0.8248f;
constexpr float GYRO_BIAS_DPS_Y =  0.1288f;
constexpr float GYRO_BIAS_DPS_Z = -0.5285f;

// Magnetometer calibration
constexpr float MAG_OFF_X = 0.0f;
constexpr float MAG_OFF_Y = -46.7f;
constexpr float MAG_OFF_Z = -25.2f;
constexpr float MAG_SCALE_X = 0.984f;
constexpr float MAG_SCALE_Y = 1.05f;
constexpr float MAG_SCALE_Z = 1.0f;

// ========================================================
//         FLEX SENSORS & STATISTICAL THRESHOLDS
// ========================================================
constexpr int NUM_FLEX = 5; 
int flexPins[NUM_FLEX] = {A0, A2, A1, A3, A7}; 
float R_FIXED[NUM_FLEX]  = {47000.0f, 47000.0f, 47000.0f, 47000.0f, 47000.0f};
constexpr float V_IN = 3.3f;
float flexR[NUM_FLEX];

// Statistically calculated Thresholds
float THRESHOLDS[NUM_FLEX][3] = {
  {38406.8f, 49937.3f, 75488.9f},
  {20396.7f, 27556.0f, 44790.9f},
  {22969.2f, 30980.6f, 47308.5f},
  {28157.5f, 32627.5f, 42820.8f},
  {13922.7f, 17090.8f, 26576.7f}
};

// Expanded Hysteresis Matrix 
float HYSTERESIS[NUM_FLEX][3] = {
  {1878.9f, 9429.3f, 14966.5f},
  {1488.4f, 5537.5f, 11153.6f},
  {3367.6f, 3961.3f, 12128.7f},
  {3310.0f,  935.5f,  9082.1f},
  { 927.4f, 2185.0f,  7134.9f}
};

// Memory array for the state machine
int currentStates[NUM_FLEX] = {0, 0, 0, 0, 0};

// ========================================================
//                    DATA STRUCTURES
// ========================================================
struct Vec3 { float x, y, z; };
struct Orientation { float roll, pitch, yaw; };

Orientation ori{0,0,0};
Orientation ref{0,0,0};

unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;
constexpr unsigned long SEND_PERIOD_MS = 100;

// ========================================================
//                      HELPERS
// ========================================================
float wrapAngle(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void readIMU(Vec3 &acc, Vec3 &gyr, Vec3 &mag) {
  if (imu.dataReady()) {
    imu.getAGMT(); 
    acc = { imu.accX() * ACC_MG_TO_G, imu.accY() * ACC_MG_TO_G, imu.accZ() * ACC_MG_TO_G };
    gyr = { imu.gyrX() - GYRO_BIAS_DPS_X, imu.gyrY() - GYRO_BIAS_DPS_Y, imu.gyrZ() - GYRO_BIAS_DPS_Z };
    mag = { (imu.magX() - MAG_OFF_X) * MAG_SCALE_X, (imu.magY() - MAG_OFF_Y) * MAG_SCALE_Y, (imu.magZ() - MAG_OFF_Z) * MAG_SCALE_Z };
  }
}

Orientation accelOrientation(const Vec3 &a) {
  return { atan2(a.y, a.z) * RAD_TO_DEG, atan2(-a.x, sqrt(a.y*a.y + a.z*a.z)) * RAD_TO_DEG, 0.0f };
}

float magYaw(const Vec3 &m, float roll, float pitch) {
  float r = roll  * DEG_TO_RAD;
  float p = pitch * DEG_TO_RAD;
  float mx = m.x * cos(p) + m.z * sin(p);
  float my = m.x * sin(r)*sin(p) + m.y*cos(r) - m.z*sin(r)*cos(p);
  return wrapAngle(atan2(-my, mx) * RAD_TO_DEG);
}

void updateOrientation(float dt, Vec3 &acc, Vec3 &gyr, Vec3 &mag) {
  readIMU(acc, gyr, mag);
  Orientation accOri = accelOrientation(acc);
  ori.roll  += gyr.x * dt;
  ori.pitch += gyr.y * dt;
  ori.yaw   += gyr.z * dt;
  float yawMag = magYaw(mag, ori.roll, ori.pitch);
  ori.roll  = wrapAngle(ALPHA_RP  * ori.roll  + (1 - ALPHA_RP)  * accOri.roll);
  ori.pitch = wrapAngle(ALPHA_RP  * ori.pitch + (1 - ALPHA_RP)  * accOri.pitch);
  ori.yaw   = wrapAngle(ALPHA_YAW * ori.yaw   + (1 - ALPHA_YAW) * yawMag);
}

// ========================================================
//    FLEX DETECTION (WITH HYSTERESIS STATE MACHINE)
// ========================================================
uint16_t readFlexBits() { 
  uint16_t flexData = 0;
  Serial.print("(r0, r1, r2, r3, r4)=(");
  
  for (int i = 0; i < NUM_FLEX; i++) {
    long sum = 0;
    for (int k = 0; k < 5; k++) sum += analogRead(flexPins[i]);
    
    float v = (sum / 5.0f) * (V_IN / 4095.0f);
    float r = 0.0f;
    
    // Safety check to prevent infinity
    if (v > 0.01f) {
      if (v >= V_IN - 0.01f) r = 1000000.0f; 
      else r = R_FIXED[i] * (v / (V_IN - v));
    }
    flexR[i] = r;

    // Hysteresis State Machine Evaluation (2D Matrix update)
    int s = currentStates[i];

    // Look UP a state, using the hysteresis for the current gap
    while (s < 3 && r > (THRESHOLDS[i][s] + HYSTERESIS[i][s])) {
        s++;
    }
    // Look DOWN a state, using the hysteresis for the gap below
    while (s > 0 && r < (THRESHOLDS[i][s - 1] - HYSTERESIS[i][s - 1])) {
        s--;
    }
    
    currentStates[i] = s;
    
    // Pack the 2 bits into the 16-bit integer
    flexData |= (s << (i * 2));
    
    Serial.print(r, 0);  
    if (i < NUM_FLEX - 1) Serial.print(", ");
  }
  Serial.println(")");
  return flexData;
}

uint8_t encodeAxis(float angle, float th) {
  if (angle >  th) return 0b01;
  if (angle < -th) return 0b10;
  return 0b00;
}

// ========================================================
//                          SETUP
// ========================================================
void setup() {
  Serial.begin(115200);
  while (!Serial); 
  Serial.println("Glove System - Serial Only (10-bit Flex + IMU)");

  Wire.begin();
  Wire.setClock(400000);

  imu.begin(Wire, AD0_VAL);
  while (imu.status != ICM_20948_Stat_Ok) {
    Serial.println("IMU init failed! Retrying...");
    imu.begin(Wire, AD0_VAL);
    delay(500);
  }

  Vec3 acc, gyr, mag;
  readIMU(acc, gyr, mag);
  ori = accelOrientation(acc);
  ori.yaw = magYaw(mag, ori.roll, ori.pitch);
  ref = ori;

  lastTime = micros();
}

// ========================================================
//                          LOOP
// ========================================================
void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;
  lastTime = now;
  if (dt <= 0.0f || dt > MAX_DT) dt = 0.01f;

  Vec3 acc, gyr, mag;
  updateOrientation(dt, acc, gyr, mag);

  unsigned long nowMs = millis();
  if (nowMs - lastPrintTime >= SEND_PERIOD_MS) {
    lastPrintTime = nowMs;

    float relRoll  = wrapAngle(ori.roll  - ref.roll);
    float relPitch = wrapAngle(ori.pitch - ref.pitch);

    uint16_t flexData = readFlexBits(); // 10 bits used 
    uint8_t rollBits = encodeAxis(relRoll,  ROLL_TH);
    uint8_t pitchBits = encodeAxis(relPitch, PITCH_TH);
    uint8_t imuByte = (rollBits << 2) | pitchBits; // 4 bits used

    // Split the 16-bit flex data
    // Masking with 0x03 ensures we only grab the top 2 bits (Finger 4)
    uint8_t flexHigh = (flexData >> 8) & 0x03; 
    uint8_t flexLow  = flexData & 0xFF; // Bottom 8 bits (Fingers 0-3)

    // --- MERGE FLEX_H AND IMU ---
    // Shift flexHigh to bits 4 and 5, and insert imuByte into bits 0 to 3
    uint8_t mergedByte = (flexHigh << 4) | imuByte;

    // --- 1. Print Raw Binary Data (START/END bytes removed) ---
    Serial.print("RAW -> [MERGED_FLEX_H_IMU: ");  
    Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(mergedByte));   
    Serial.print("] [FLEX_L: ");  
    Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(flexLow));    
    Serial.print("]\n");

    // --- 2. Parse and Print Human-Readable Data ---
    uint8_t f0 = flexData & 0x03;          
    uint8_t f1 = (flexData >> 2) & 0x03;   
    uint8_t f2 = (flexData >> 4) & 0x03;   
    uint8_t f3 = (flexData >> 6) & 0x03;   
    uint8_t f4 = (flexData >> 8) & 0x03; 

    Serial.print("PARSED -> Fingers (F4,F3,F2,F1,F0): ");
    Serial.print(f4); Serial.print(", ");
    Serial.print(f3); Serial.print(", ");
    Serial.print(f2); Serial.print(", ");
    Serial.print(f1); Serial.print(", ");
    Serial.print(f0);

    Serial.print(" | IMU: Roll=");
    if (rollBits == 0b01) Serial.print("+");
    else if (rollBits == 0b10) Serial.print("-");
    else Serial.print("0");

    Serial.print(", Pitch=");
    if (pitchBits == 0b01) Serial.print("+");
    else if (pitchBits == 0b10) Serial.print("-");
    else Serial.println("0");

    Serial.println("\n--------------------------------------------------");
  }
}