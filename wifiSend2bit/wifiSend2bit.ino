#include <Wire.h>
#include <ICM_20948.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

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
//                  NETWORK SETUP
// ========================================================
const char* ssid = "YP";
const char* password = "ptyair101";
const char* piHostname = "libsonpi";
const int udpPort = 4210;

WiFiUDP udp;
IPAddress resolvedPiIP;

bool lastPacketWasZero = false;

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

struct Vec3 { float x, y, z; };
struct Orientation { float roll, pitch, yaw; };

Orientation ori{0,0,0};
Orientation ref{0,0,0};

unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;
constexpr unsigned long SEND_PERIOD_MS = 100;

// ========================================================
//         FLEX SENSORS & STATISTICAL THRESHOLDS
// ========================================================
constexpr int NUM_FLEX = 5; 
// REORDERED PINS: f0(A3), f1(A0), f2(A2), f3(A7), f4(A1)
int flexPins[NUM_FLEX] = {A3, A0, A2, A7, A1}; 
float R_FIXED[NUM_FLEX]  = {47000.0f, 47000.0f, 47000.0f, 47000.0f, 47000.0f};
constexpr float V_IN = 3.3f;
float flexR[NUM_FLEX];

// Statistically calculated Thresholds (Reordered to match new pins)
float THRESHOLDS[NUM_FLEX][3] = {
  {28157.5f, 32627.5f, 42820.8f}, // f0 (A3)
  {38406.8f, 49937.3f, 75488.9f}, // f1 (A0)
  {20396.7f, 27556.0f, 44790.9f}, // f2 (A2)
  {13922.7f, 17090.8f, 26576.7f}, // f3 (A7)
  {22969.2f, 30980.6f, 47308.5f}  // f4 (A1)
};

// Expanded Hysteresis Matrix (Reordered to match new pins)
float HYSTERESIS[NUM_FLEX][3] = {
  {3310.0f,  935.5f,  9082.1f},   // f0 (A3)
  {1878.9f, 9429.3f, 14966.5f},   // f1 (A0)
  {1488.4f, 5537.5f, 11153.6f},   // f2 (A2)
  { 927.4f, 2185.0f,  7134.9f},   // f3 (A7)
  {3367.6f, 3961.3f, 12128.7f}    // f4 (A1)
};

// Memory array for the state machine
int currentStates[NUM_FLEX] = {0, 0, 0, 0, 0};

// ========================================================
//                  MODE TOGGLE GESTURE SETUP
// ========================================================
unsigned long gestureStartTime = 0;
bool gestureActive = false;
bool gestureProcessed = false;
uint8_t modeBit = 0; // The new bit to tell the robot how to parse data

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

    // Look UP a state
    while (s < 3 && r > (THRESHOLDS[i][s] + HYSTERESIS[i][s])) s++;
    // Look DOWN a state
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
//                          SETUP
// ========================================================
void setup() {
  Serial.begin(115200);
  // while (!Serial); // Commented out to prevent hang when running standalone
  Serial.println("Glove System - Reordered 5-Finger UDP Mode + Toggle Bit");

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

  // --- WIFI & UDP SETUP ---
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());

  MDNS.begin("esp32-glove");
  resolvedPiIP = MDNS.queryHost(piHostname);
  while (resolvedPiIP.toString() == "0.0.0.0") {
    delay(1000);
    resolvedPiIP = MDNS.queryHost(piHostname);
    Serial.println("Resolving Pi IP...");
  }

  Serial.print("Resolved Pi IP: ");
  Serial.println(resolvedPiIP);
  Serial.print("UDP destination port: ");
  Serial.println(udpPort);

  udp.begin(udpPort);
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

    uint16_t flexData = readFlexBits();
    uint8_t rollBits = encodeAxis(relRoll,  ROLL_TH);
    uint8_t pitchBits = encodeAxis(relPitch, PITCH_TH);
    uint8_t imuByte = (rollBits << 2) | pitchBits;

    // --- GESTURE DETECTION FOR MODE BIT ---
    // User applied all fingers (state 3) except f1 and f2 (state 0)
    bool isGesture = (currentStates[0] >= 1 && currentStates[3] >= 1 && currentStates[4] >= 1 && 
                      currentStates[1] == 0 && currentStates[2] == 0);

    if (isGesture) {
      if (!gestureActive) {
        gestureActive = true;
        gestureStartTime = nowMs;
        gestureProcessed = false;
      } else if (!gestureProcessed && (nowMs - gestureStartTime >= 2000)) {
        modeBit = (modeBit == 0) ? 1 : 0; // Toggle the bit
        gestureProcessed = true; // Prevent rapid toggling
        Serial.print("\n>>> MODE BIT TOGGLED TO: ");
        Serial.println(modeBit);
      }
    } else {
      gestureActive = false;
      gestureProcessed = false;
    }

    // Split the 16-bit flex data
    uint8_t flexHigh = (flexData >> 8) & 0x03; // Top 2 bits (Finger 4)
    uint8_t flexLow  = flexData & 0xFF;        // Bottom 8 bits (Fingers 0-3)

    // Merge modeBit (bit 6), flexHigh (bits 4-5), and imuByte (bits 0-3)
    uint8_t mergedByte = (modeBit << 6) | (flexHigh << 4) | imuByte;

    // Output strictly for Serial Debugging
    Serial.print("IMU: R="); Serial.print(relRoll, 1);
    Serial.print(" P="); Serial.print(relPitch, 1);
    Serial.print(" | Mode: "); Serial.print(modeBit);
    Serial.print(" | Merged: "); Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(mergedByte));
    Serial.print(" | Low: "); Serial.printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(flexLow));
    Serial.println();

    // Mask out the modeBit when checking for a zero packet to save battery when the hand is flat
    bool isZeroPacket = ((mergedByte & 0x3F) == 0x00 && flexLow == 0x00);

    // --- UDP TRANSMISSION (Raw 2-Byte Payload) ---
    if (!(isZeroPacket && lastPacketWasZero)) {
      // Create a 2-byte buffer, dropping the AA/55 boundary bytes
      uint8_t buf[2] = { mergedByte, flexLow };
      
      udp.beginPacket(resolvedPiIP, udpPort);
      udp.write(buf, sizeof(buf));
      udp.endPacket();
      
      lastPacketWasZero = isZeroPacket;
    }
  }
}