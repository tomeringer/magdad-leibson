#include <Wire.h>
#include <ICM_20948.h>
#include "driver/rmt.h"

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
//          RF TRANSMITTER SETUP
// ========================================================
#define TX_PIN 5 
#define HALF_BIT 500 // תזמון יציב מוכח לאוויר

// ========================================================
//                  IMU SETUP
// ========================================================
ICM_20948_I2C imu;
#define AD0_VAL 0

constexpr float ACC_MG_TO_G = 0.001f;
constexpr float ALPHA_RP  = 0.98f;
constexpr float ALPHA_YAW = 0.98f;
constexpr float MAX_DT = 0.1f; // 100ms kill-switch for IMU drift

constexpr float ROLL_TH  = 45.0f;
constexpr float PITCH_TH = 30.0f;

constexpr float GYRO_BIAS_DPS_X = -0.8248f;
constexpr float GYRO_BIAS_DPS_Y =  0.1288f;
constexpr float GYRO_BIAS_DPS_Z = -0.5285f;
constexpr float MAG_OFF_X = 0.0f;
constexpr float MAG_OFF_Y = -46.7f;
constexpr float MAG_OFF_Z = -25.2f;
constexpr float MAG_SCALE_X = 0.984f;
constexpr float MAG_SCALE_Y = 1.05f;
constexpr float MAG_SCALE_Z = 1.0f;

struct Vec3 { float x, y, z; };
struct Orientation { float roll, pitch, yaw; };

Orientation ori{0,0,0};
Orientation ref{0,0,0};

unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;

// === INCREASED SEND RATE TO MATCH NON-BLOCKING RF ===
constexpr unsigned long SEND_PERIOD_MS = 33; // Fires ~30 times a second

bool lastPacketWasZero = false;

// ========================================================
//         FLEX SENSORS & HYSTERESIS
// ========================================================
constexpr int NUM_FLEX = 5; 
// === סידור האצבעות המבוקש ===
int flexPins[NUM_FLEX] = {A3, A0, A2, A7, A1}; 

float R_FIXED[NUM_FLEX]  = {47000.0f, 47000.0f, 47000.0f, 47000.0f, 47000.0f};
constexpr float V_IN = 3.3f;
float flexR[NUM_FLEX];

// הותאם לסדר החדש של הפינים
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

int currentStates[NUM_FLEX] = {0, 0, 0, 0, 0};

unsigned long gestureStartTime = 0;
bool gestureActive = false;
bool gestureProcessed = false;
uint8_t modeBit = 0; 

// ========================================================
//                       HELPERS
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

uint16_t readFlexBits() { 
  uint16_t flexData = 0;
  for (int i = 0; i < NUM_FLEX; i++) {
    long sum = 0;
    for (int k = 0; k < 5; k++) sum += analogRead(flexPins[i]);
    float v = (sum / 5.0f) * (V_IN / 4095.0f);
    float r = 0.0f;
    if (v > 0.01f) {
      if (v >= V_IN - 0.01f) r = 1000000.0f; 
      else r = R_FIXED[i] * (v / (V_IN - v));
    }
    flexR[i] = r;

    int s = currentStates[i];
    while (s < 3 && r > (THRESHOLDS[i][s] + HYSTERESIS[i][s])) s++;
    while (s > 0 && r < (THRESHOLDS[i][s - 1] - HYSTERESIS[i][s - 1])) s--;
    
    currentStates[i] = s;
    flexData |= (s << (i * 2));
  }
  return flexData;
}

uint8_t encodeAxis(float angle, float th) {
  if (angle >  th) return 0b01;
  if (angle < -th) return 0b10;
  return 0b00;
}

// ========================================================
//            RF TRANSMIT FUNCTION (Compact 3-Byte)
// ========================================================
void transmitCompactData(uint8_t mergedByte, uint8_t flexLow) {
  rmt_item32_t items[80]; 
  int idx = 0;

  // 1. חימום (Preamble)
  for(int i = 0; i < 6; i++) {
    items[idx++] = {{{ 400, 1, 400, 0 }}}; 
  }

  // 2. סנכרון
  items[idx++] = {{{ 1000, 1, 1000, 0 }}};

  // 3. ביט התחלה
  items[idx++] = {{{ HALF_BIT, 1, HALF_BIT, 0 }}};

  // 4. בניית המטען (24 ביטים)
  uint8_t checksum = (mergedByte + flexLow) & 0xFF;
  uint32_t payload = ((uint32_t)mergedByte << 16) | ((uint32_t)flexLow << 8) | checksum;

  for (int i = 23; i >= 0; i--) {
    if ((payload >> i) & 1) items[idx++] = {{{ HALF_BIT, 1, HALF_BIT, 0 }}}; 
    else items[idx++] = {{{ HALF_BIT, 0, HALF_BIT, 1 }}}; 
  }
  
  items[idx++] = {{{ 0, 0, 0, 0 }}};

  // === FIXED: Single fire, NO DELAY, non-blocking ===
  rmt_write_items(RMT_CHANNEL_0, items, idx, true);
}

// ========================================================
//                          SETUP
// ========================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } 
  Serial.println("Glove System - Fast Compact RF Mode (Drive Commands Only)");

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

  rmt_config_t config;
  config.rmt_mode = RMT_MODE_TX;
  config.channel = RMT_CHANNEL_0;
  config.gpio_num = (gpio_num_t)TX_PIN;
  config.mem_block_num = 1;
  config.clk_div = 80; 
  config.tx_config.loop_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  rmt_config(&config);
  rmt_driver_install(config.channel, 0, 0);

  lastTime = micros();
}

// ========================================================
//                          LOOP
// ========================================================
void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;
  lastTime = now;
  
  // IMU protection check
  if (dt <= 0.0f || dt > MAX_DT) dt = 0.01f;

  Vec3 acc, gyr, mag;
  updateOrientation(dt, acc, gyr, mag);

  unsigned long nowMs = millis();
  if (nowMs - lastPrintTime >= SEND_PERIOD_MS) {
    lastPrintTime = nowMs;

    float relRoll  = wrapAngle(ori.roll  - ref.roll);
    float relPitch = wrapAngle(ori.pitch - ref.pitch);

    uint16_t flexData = readFlexBits();
    
    // === היפוך כיווני הנסיעה (כפי שביקשת) ===
    uint8_t rollBits = encodeAxis(-relRoll,  ROLL_TH);
    uint8_t pitchBits = encodeAxis(-relPitch, PITCH_TH);
    uint8_t imuByte = (rollBits << 2) | pitchBits;

    bool isGesture = (currentStates[0] >= 1 && currentStates[3] >= 1 && currentStates[4] >= 1 && 
                      currentStates[1] == 0 && currentStates[2] == 0);

    if (isGesture) {
      if (!gestureActive) {
        gestureActive = true;
        gestureStartTime = nowMs;
        gestureProcessed = false;
      } else if (!gestureProcessed && (nowMs - gestureStartTime >= 2000)) {
        modeBit = (modeBit == 0) ? 1 : 0; 
        gestureProcessed = true; 
        Serial.print("\n>>> MODE BIT TOGGLED TO: ");
        Serial.println(modeBit);
      }
    } else {
      gestureActive = false;
      gestureProcessed = false;
    }

    uint8_t flexHigh = (flexData >> 8) & 0x03; 
    uint8_t flexLow  = flexData & 0xFF;        

    uint8_t mergedByte = (modeBit << 6) | (flexHigh << 4) | imuByte;

    bool isZeroPacket = ((mergedByte & 0x3F) == 0x00 && flexLow == 0x00);

    // Output strictly for Serial Debugging
    Serial.print("IMU: R="); Serial.print(relRoll, 1);
    Serial.print(" P="); Serial.print(relPitch, 1);
    Serial.print(" | Mode: "); Serial.print(modeBit);
    Serial.print(" | Merged: "); Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(mergedByte));
    Serial.print(" | Low: "); Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(flexLow));
    Serial.println();

    // חסכון בסוללה כשהיד שטוחה, פרט לפעם הראשונה
    if (!(isZeroPacket && lastPacketWasZero)) {
      
      // שידור אלחוטי קומפקטי פעם אחת, מהיר וללא עיכובים
      transmitCompactData(mergedByte, flexLow);
      
      lastPacketWasZero = isZeroPacket;
    }
  }
}