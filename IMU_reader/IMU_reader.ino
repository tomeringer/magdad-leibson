#include <Wire.h>
#include <ICM_20948.h>

ICM_20948_I2C imu;

#define AD0_VAL 0   // AD0 = GND -> I2C address 0x68

// ======================= CONSTANTS =========================
constexpr float ACC_MG_TO_G = 0.001f;

constexpr float ALPHA_RP  = 0.98f;
constexpr float ALPHA_YAW = 0.98f;

constexpr float MAX_DT = 0.1f;

// ================= ORIENTATION THRESHOLDS =================
// degrees
constexpr float ROLL_TH  = 10.0f;
constexpr float PITCH_TH = 10.0f;

// ================= MAG CALIBRATION =================
constexpr float MAG_OFF_X   = 0.0f;
constexpr float MAG_OFF_Y   = -46.7f;
constexpr float MAG_OFF_Z   = -25.2f;

constexpr float MAG_SCALE_X = 0.984f;
constexpr float MAG_SCALE_Y = 1.05f;
constexpr float MAG_SCALE_Z = 1.0f;

// ================= GYRO BIAS (deg/s) =================
constexpr float GYRO_BIAS_DPS_X = -0.8248f;
constexpr float GYRO_BIAS_DPS_Y =  0.1288f;
constexpr float GYRO_BIAS_DPS_Z = -0.5285f;

// ======================= DATA TYPES =============================
struct Vec3 {
  float x, y, z;
};

struct Orientation {
  float roll;
  float pitch;
  float yaw;
};

// ======================= STATE =============================
Orientation ori{0.0f, 0.0f, 0.0f};
Orientation ref{0.0f, 0.0f, 0.0f};

unsigned long lastTime = 0;

// ===========================================================
//                          UTILS
// ===========================================================
float wrapAngle(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// 2-bit encoder
// 01 -> angle > +th
// 10 -> angle < -th
// 00 -> neutral
uint8_t encodeAxis(float angle, float th) {
  if (angle >  th) return 0b01;
  if (angle < -th) return 0b10;
  return 0b00;
}

// ===========================================================
//                        SENSORS
// ===========================================================
void readSensors(Vec3 &acc, Vec3 &gyr, Vec3 &mag) {
  imu.getAGMT();

  acc.x = imu.accX() * ACC_MG_TO_G;
  acc.y = imu.accY() * ACC_MG_TO_G;
  acc.z = imu.accZ() * ACC_MG_TO_G;

  gyr.x = imu.gyrX() - GYRO_BIAS_DPS_X;
  gyr.y = imu.gyrY() - GYRO_BIAS_DPS_Y;
  gyr.z = imu.gyrZ() - GYRO_BIAS_DPS_Z;

  mag.x = (imu.magX() - MAG_OFF_X) * MAG_SCALE_X;
  mag.y = (imu.magY() - MAG_OFF_Y) * MAG_SCALE_Y;
  mag.z = (imu.magZ() - MAG_OFF_Z) * MAG_SCALE_Z;
}

// ===========================================================
//                     ORIENTATION MATH
// ===========================================================
Orientation computeAccelAngles(const Vec3 &acc) {
  Orientation o;
  o.roll  = atan2(acc.y, acc.z) * RAD_TO_DEG;
  o.pitch = atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * RAD_TO_DEG;
  o.yaw   = 0.0f;
  return o;
}

float computeYawFromMag(const Vec3 &mag, float roll, float pitch) {
  float r = roll  * DEG_TO_RAD;
  float p = pitch * DEG_TO_RAD;

  float mx_h = mag.x * cos(p) + mag.z * sin(p);
  float my_h = mag.x * sin(r) * sin(p)
             + mag.y * cos(r)
             - mag.z * sin(r) * cos(p);

  return wrapAngle(atan2(-my_h, mx_h) * RAD_TO_DEG);
}

void integrateGyro(Orientation &o, const Vec3 &gyr, float dt) {
  o.roll  += gyr.x * dt;
  o.pitch += gyr.y * dt;
  o.yaw   += gyr.z * dt;
}

// ===========================================================
//                       FILTER
// ===========================================================
void applyComplementaryFilter(
  Orientation &o,
  const Orientation &accel,
  float yaw_mag_abs
) {
  o.roll  = ALPHA_RP  * o.roll  + (1.0f - ALPHA_RP)  * accel.roll;
  o.pitch = ALPHA_RP  * o.pitch + (1.0f - ALPHA_RP)  * accel.pitch;
  o.yaw   = ALPHA_YAW * o.yaw   + (1.0f - ALPHA_YAW) * yaw_mag_abs;

  o.roll  = wrapAngle(o.roll);
  o.pitch = wrapAngle(o.pitch);
  o.yaw   = wrapAngle(o.yaw);
}

// ===========================================================
//                    REFERENCE HANDLING
// ===========================================================
void setReference(const Orientation &o) {
  ref = o;
}

Orientation applyReference(const Orientation &o) {
  Orientation r;
  r.roll  = wrapAngle(o.roll  - ref.roll);
  r.pitch = wrapAngle(o.pitch - ref.pitch);
  r.yaw   = wrapAngle(o.yaw   - ref.yaw);
  return r;
}

// ===========================================================
//                          SETUP
// ===========================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  imu.begin(Wire, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.println("IMU init failed");
    while (1);
  }

  delay(200);

  Vec3 acc, gyr, mag;
  readSensors(acc, gyr, mag);

  Orientation accOri = computeAccelAngles(acc);
  ori = accOri;
  ori.yaw = computeYawFromMag(mag, ori.roll, ori.pitch);

  setReference(ori);

  lastTime = micros();
  Serial.println("IMU ready, reference set.");
}

// ===========================================================
//                           LOOP
// ===========================================================
void loop() {
  if (!imu.dataReady()) return;

  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;
  lastTime = now;
  if (dt <= 0.0f || dt > MAX_DT) return;

  Vec3 acc, gyr, mag;
  readSensors(acc, gyr, mag);

  Orientation accOri = computeAccelAngles(acc);

  integrateGyro(ori, gyr, dt);

  float yaw_mag_abs = computeYawFromMag(mag, ori.roll, ori.pitch);

  applyComplementaryFilter(ori, accOri, yaw_mag_abs);

  Orientation rel = applyReference(ori);

  // ================= THRESHOLD ENCODING =================
  uint8_t roll_code  = encodeAxis(rel.roll,  ROLL_TH);
  uint8_t pitch_code = encodeAxis(rel.pitch, PITCH_TH);

  // Pack into one byte:
  // bits 3-2: roll
  // bits 1-0: pitch
  uint8_t packed =
      (roll_code  << 2) |
      (pitch_code);

  // ================= OUTPUT =================
  Serial.println("====== IMU ORIENTATION ======");
  Serial.print("Roll (deg):  ");  Serial.println(rel.roll, 3);
  Serial.print("Pitch (deg): "); Serial.println(rel.pitch, 3);
  Serial.print("yaw (deg): "); Serial.println(rel.yaw, 3);

  Serial.print("Packed byte: 0b");
  Serial.println(packed, BIN);
  Serial.println();

  delay(5);
}
