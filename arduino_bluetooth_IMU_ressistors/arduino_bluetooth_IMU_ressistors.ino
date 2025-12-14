#include <Wire.h>
#include <ICM_20948.h>

// --------------------------------------------------------
//                         IMU SETUP
// --------------------------------------------------------
ICM_20948_I2C imu;
#define AD0_VAL 0  

// Bluetooth framing bytes
byte START = 0xAA;
byte END   = 0x55;

// Store last sent packet to avoid unnecessary transmissions
byte last_packet1 = 0xFF;
byte last_packet2 = 0xFF;

const float ACC_MG_TO_G = 0.001f;
const float G_TO_MS2 = 9.80665f;

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

unsigned long lastTime = 0;

const float ALPHA = 0.98;
const float ALPHA_YAW = 0.98;

// --------------------------------------------------------
//                   RESISTOR SENSOR SETUP
// --------------------------------------------------------
float R_DIVIDER[] ={5000,5000,5000,64000};
float V_IN = 5.0;
int pins[] = {A0, A1, A2, A3};
float resistances[4];

// --------------------------------------------------------
//                   THRESHOLDS
// --------------------------------------------------------
float R_THRESH[4] = {30000, 30000, 11000, 650000};

// IMU thresholds (angle in degrees)
float ROLL_LOW  = -10, ROLL_HIGH  = 10;
float PITCH_LOW = -10, PITCH_HIGH = 10;
float YAW_LOW   = -20, YAW_HIGH   = 20;

// --------------------------------------------------------
//                         SETUP
// --------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
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


// --------------------------------------------------------
//                         LOOP
// --------------------------------------------------------
void loop() {

  // ----------- Read resistors -----------
  for (int i = 0; i < 4; i++) {
    int adcVal = analogRead(pins[i]);
    if (adcVal == 0) adcVal = 1;

    float voltage = adcVal * (V_IN / 1023.0);
    resistances[i] = R_DIVIDER[i] * (voltage / (V_IN - voltage));
  }

  // ----------- Read IMU always -----------
  imu.getAGMT();

  float ax = imu.accX() * ACC_MG_TO_G;
  float ay = imu.accY() * ACC_MG_TO_G;
  float az = imu.accZ() * ACC_MG_TO_G;

  float gx = imu.gyrX();
  float gy = imu.gyrY();
  float gz = imu.gyrZ();

  float mx = imu.magX();
  float my = imu.magY();
  float mz = imu.magZ();

  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;
  lastTime = now;

  float accelRoll  = atan2(ay, az) * 180.0 / PI;
  float accelPitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;

  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;

  float roll_rad = roll * PI/180.0f;
  float pitch_rad = pitch * PI/180.0f;

  float mx_h = mx * cos(pitch_rad) + mz * sin(pitch_rad);
  float my_h = mx * sin(roll_rad)*sin(pitch_rad) +
               my * cos(roll_rad) -
               mz * sin(roll_rad)*cos(pitch_rad);

  float yaw_mag = atan2(-my_h, mx_h) * 180.0f / PI;
  yaw = ALPHA_YAW * yaw + (1.0f - ALPHA_YAW) * yaw_mag;

  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;

  roll  = ALPHA * roll  + (1.0 - ALPHA) * accelRoll;
  pitch = ALPHA * pitch + (1.0 - ALPHA) * accelPitch;

  // ======================================================
  //               PACK THRESHOLD INFORMATION
  // ======================================================

  // 4 bits → resistors
  byte R_bits = 0;
  for (int i = 0; i < 4; i++) {
    if (resistances[i] > R_THRESH[i]) {
      R_bits |= (1 << i);
    }
  }

  // Each angle contributes 2 bits (low, high)
  byte imu_bits = 0;

  // Roll
  if (roll < ROLL_LOW)  imu_bits |= (1 << 0);
  if (roll > ROLL_HIGH) imu_bits |= (1 << 1);

  // Pitch
  if (pitch < PITCH_LOW)  imu_bits |= (1 << 2);
  if (pitch > PITCH_HIGH) imu_bits |= (1 << 3);

  // Yaw
  if (yaw < YAW_LOW)  imu_bits |= (1 << 4);
  if (yaw > YAW_HIGH) imu_bits |= (1 << 5);

  // Packet = 2 bytes
  byte packet1 = R_bits;     // 4 bits used
  byte packet2 = imu_bits;   // 6 bits used

  // ======================================================
  //                BLUETOOTH PACKET SEND
  // ======================================================

  // Send only if packet changed
    last_packet1 = packet1;
    last_packet2 = packet2;


    // ---- Send over Bluetooth (HC-05) ----
    Serial1.write(START);
    Serial1.write(packet1);
    Serial1.write(packet2);
    Serial1.write(END);
  

  // ======================================================
  //               PRINT EVERYTHING
  // ======================================================
  Serial.print("R:");
  Serial.print((int)resistances[0]); Serial.print(",");
  Serial.print((int)resistances[1]); Serial.print(",");
  Serial.print((int)resistances[2]); Serial.print(",");
  Serial.print((int)resistances[3]);

  Serial.print(" | Ori:");
  Serial.print(roll, 2); Serial.print(",");
  Serial.print(pitch, 2); Serial.print(",");
  Serial.print(yaw, 2);

  Serial.print(" | p1_dec=");
  Serial.print(packet1);
  Serial.print(" p2_dec=");
  Serial.print(packet2);

  Serial.print(" | Packet: ");
  Serial.print(packet1, BIN); Serial.print(" ");
  Serial.println(packet2, BIN);


  delay(1000);
}
