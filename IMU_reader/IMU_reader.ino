#include <Wire.h>
#include <ICM_20948.h>

ICM_20948_I2C imu;

#define AD0_VAL 0  // AD0 = GND -> address 0x68

// Conversion
const float ACC_MG_TO_G = 0.001f;
const float G_TO_MS2 = 9.80665f;

// Orientation filter variables
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

unsigned long lastTime = 0;

// Complementary filter constant
const float ALPHA = 0.98;   // gyro weight (0.95–0.99 typical)

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  imu.begin(Wire, AD0_VAL);

  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.println("IMU init failed.");
    while (1);
  }

  Serial.println("IMU initialized!");
  lastTime = micros();
}

void loop() {

  if (imu.dataReady()) {

    imu.getAGMT();

    // ---------------- ACCEL ----------------
    float ax = imu.accX() * ACC_MG_TO_G;
    float ay = imu.accY() * ACC_MG_TO_G;
    float az = imu.accZ() * ACC_MG_TO_G;

    // ---------------- GYRO ----------------
    float gx = imu.gyrX();   // deg/s
    float gy = imu.gyrY();
    float gz = imu.gyrZ();

    // ---------------- Time delta ----------------
    unsigned long now = micros();
    float dt = (now - lastTime) * 1e-6f;  // convert to seconds
    lastTime = now;

    // ---------------- ACCEL ANGLES ----------------
    float accelRoll  = atan2(ay, az) * 180.0 / PI;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // ---------------- GYRO INTEGRATION ----------------
    roll  += gx * dt;
    pitch += gy * dt;
    yaw   += gz * dt;   // yaw needs gyro or magnetometer

    // ---------------- COMPLEMENTARY FILTER ----------------
    roll  = ALPHA * roll  + (1.0 - ALPHA) * accelRoll;
    pitch = ALPHA * pitch + (1.0 - ALPHA) * accelPitch;

    // ---------------- PRINT ----------------
    Serial.println("====== IMU ORIENTATION ======");
    Serial.print("Roll (deg):  "); Serial.println(roll, 3);
    Serial.print("Pitch (deg): "); Serial.println(pitch, 3);
    Serial.print("Yaw (deg):   "); Serial.println(yaw, 3);

    Serial.println();
  }

  delay(5);
}
