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
const float ALPHA_YAW = 0.98;   // gyro weight, tune 0.95–0.995


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

    // ----- ACCEL -----
    float ax = imu.accX() * ACC_MG_TO_G;
    float ay = imu.accY() * ACC_MG_TO_G;
    float az = imu.accZ() * ACC_MG_TO_G;

    // ----- GYRO -----
    float gx = imu.gyrX();   // deg/s
    float gy = imu.gyrY();
    float gz = imu.gyrZ();

    // ----- MAGNETOMETER -----
    float mx = imu.magX();
    float my = imu.magY();
    float mz = imu.magZ();

    // ----- dt -----
    unsigned long now = micros();
    float dt = (now - lastTime) * 1e-6f;
    lastTime = now;

    // ----- ACCEL ANGLES -----
    float accelRoll  = atan2(ay, az) * 180.0 / PI;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // ----- GYRO INTEGRATION -----
    roll  += gx * dt;
    pitch += gy * dt;
    yaw   += gz * dt;   

    // ==========================================================
    //                TILT-COMPENSATED MAG YAW
    // ==========================================================

    // Convert filtered roll/pitch to radians
    float roll_rad  = roll  * PI / 180.0f;
    float pitch_rad = pitch * PI / 180.0f;

    // Tilt compensation
    float mx_h = mx * cos(pitch_rad) + mz * sin(pitch_rad);
    float my_h = mx * sin(roll_rad) * sin(pitch_rad)
               + my * cos(roll_rad)
               - mz * sin(roll_rad) * cos(pitch_rad);

    float yaw_mag = atan2(-my_h, mx_h) * 180.0f / PI;

    // ==========================================================
    //                COMPLEMENTARY FILTER (for yaw)
    // ==========================================================
    const float ALPHA_YAW = 0.98;   // gyro weight, tune 0.95–0.995

    yaw = ALPHA_YAW * yaw + (1.0f - ALPHA_YAW) * yaw_mag;

    // Optional: wrap yaw
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;


    // ----- COMPLEMENTARY FILTER FOR ROLL/PITCH -----
    roll  = ALPHA * roll  + (1.0 - ALPHA) * accelRoll;
    pitch = ALPHA * pitch + (1.0 - ALPHA) * accelPitch;

    // ----- PRINT -----
    Serial.println("====== IMU ORIENTATION ======");
    Serial.print("Roll (deg):  "); Serial.println(roll, 3);
    Serial.print("Pitch (deg): "); Serial.println(pitch, 3);
    Serial.print("Yaw (deg):   "); Serial.println(yaw, 3);
    Serial.println();
  }

  delay(5);
}
