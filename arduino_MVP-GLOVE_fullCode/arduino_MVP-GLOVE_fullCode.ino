#include <Wire.h>
#include <ICM_20948.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h> // Dynamic IP discovery library

// ========================================================
//                 NETWORK SETUP
// ========================================================
const char* ssid = "YP";
const char* password = "ptyair101";
const char* piHostname = "libsonpi"; // Your Pi's hostname (without .local)
const int udpPort = 4210;

WiFiUDP udp;
IPAddress resolvedPiIP; // Will store the IP found via mDNS

// ========================================================
//                 PACKET FRAMING
// ========================================================
constexpr uint8_t START_BYTE = 0xAA;
constexpr uint8_t END_BYTE   = 0x55;

// ========================================================
//                  IMU SETUP & CONSTANTS
// ========================================================
ICM_20948_I2C imu;
#define AD0_VAL 0

constexpr float ACC_MG_TO_G = 0.001f;
constexpr float ALPHA_RP  = 0.98f;
constexpr float ALPHA_YAW = 0.98f;
constexpr float MAX_DT = 0.1f;

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

// ========================================================
//                  FLEX SENSOR SETUP
// ========================================================
constexpr int NUM_FLEX = 4;
int flexPins[NUM_FLEX] = {A0, A1, A2, A3};
float R_FIXED[NUM_FLEX]  = {47000, 47000, 47000, 47000};
float R_THRESH[NUM_FLEX] = {40000, 22000, 32000, 22000};
constexpr float V_IN = 3.3;
float flexR[NUM_FLEX];

// ========================================================
//                   DATA STRUCTURES
// ========================================================
struct Vec3 { float x, y, z; };
struct Orientation { float roll, pitch, yaw; };

Orientation ori{0,0,0};
Orientation ref{0,0,0};

unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;
constexpr unsigned long SEND_PERIOD_MS = 50; //20Hz 

// ========================================================
//                      IMU LOGIC
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

void updateOrientation(float dt) {
  Vec3 acc, gyr, mag;
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
//                      FLEX & PACKET
// ========================================================
uint8_t readFlexBits() {
  uint8_t bits = 0;
  for (int i = 0; i < NUM_FLEX; i++) {
    long sum = 0;
    for (int k = 0; k < 5; k++) sum += analogRead(flexPins[i]);
    float v = (sum / 5.0f) * (V_IN / 4095.0f);
    flexR[i] = (v > 0.01) ? R_FIXED[i] * (v / (V_IN - v)) : 0;
    if (flexR[i] > R_THRESH[i]) bits |= (1 << i);
  }
  return bits;
}

uint8_t encodeAxis(float angle, float th) {
  if (angle >  th) return 0b01;
  if (angle < -th) return 0b10;
  return 0b00;
}

// ========================================================
//                         SETUP
// ========================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Glove Starts");
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 5000); 

  Serial.println("\n--- Unified System Starting with mDNS ---");

  Wire.begin();
  Wire.setClock(400000);

  imu.begin(Wire, AD0_VAL);
  while (imu.status != ICM_20948_Stat_Ok) {
    Serial.println("IMU init failed! tries again");
    imu.begin(Wire, AD0_VAL);

  }

  Vec3 acc, gyr, mag;
  readIMU(acc, gyr, mag);
  ori = accelOrientation(acc);
  ori.yaw = magYaw(mag, ori.roll, ori.pitch);
  ref = ori;

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  // Start mDNS discovery
  if (!MDNS.begin("esp32-glove")) {
    Serial.println("Error setting up mDNS responder!");
  }

  Serial.print("Resolving Raspberry Pi address for: ");
  Serial.println(piHostname);
  
  // Try to find the Pi's IP (timeout 5 seconds)
  resolvedPiIP = MDNS.queryHost(piHostname);
  while (resolvedPiIP.toString() == "0.0.0.0") {
    Serial.println("Pi IP not found, retrying...");
    delay(1000);
    resolvedPiIP = MDNS.queryHost(piHostname);
  }

  Serial.print("Success! Pi IP: ");
  Serial.println(resolvedPiIP);

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

  updateOrientation(dt);

  unsigned long nowMs = millis();
  if (nowMs - lastPrintTime >= SEND_PERIOD_MS) {
    lastPrintTime = nowMs;

    float relRoll  = wrapAngle(ori.roll  - ref.roll);
    float relPitch = wrapAngle(ori.pitch - ref.pitch);

    uint8_t flexBits  = readFlexBits();
    uint8_t rollBits  = encodeAxis(relRoll,  ROLL_TH);
    uint8_t pitchBits = encodeAxis(relPitch, PITCH_TH);

    uint8_t dataByte = (flexBits << 4) | (rollBits << 2) | (pitchBits);
    
    // Use the resolved IP instead of a hardcoded string
    if (resolvedPiIP.toString() != "0.0.0.0") {
      uint8_t buf[3] = { START_BYTE, dataByte, END_BYTE };
      udp.beginPacket(resolvedPiIP, udpPort);
      udp.write(buf, sizeof(buf));
      udp.endPacket();

      Serial.print("DATA=0b: ");  Serial.println(buf[1], BIN);

    }

    // Serial.print("Roll:"); Serial.print(relRoll, 1);
    // Serial.print(" Pitch:"); Serial.print(relPitch, 1);
    // Serial.print(" FlexBits:"); Serial.println(flexBits, BIN);
  }
}