#include <SPI.h>
#include "MPU9250.h"
#include <math.h>
#include <Encoder.h>
#include "KalmanFuse.h"

#define WHEEL_COUNTS_LENGTH 200

float getBikeSpeed(int32_t);
int32_t wheel_counts[WHEEL_COUNTS_LENGTH] = {0};
float g = 9.81;
float previous_phi_m = 0;
float previous_phi, previous_psi;
unsigned long iteration_counter = 0;
bool sensorfuse = 1; //if 1 we use complimentary filter if 0 we use Kalman Filter
unsigned long t1, t2;
float a; //complimentary filter constant
Encoder wheel_counter(2, 3);
MPU9250 IMU(SPI, 10);
KalmanFuse roll_kalman(0, 0, 0.0009, 0.0003, 0.5);
KalmanFuse yaw_kalman(0, 0, 0.0009, 0.0003, 0.5);

void setup()
{

  SPI.begin();
  IMU.begin();
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  t1 = micros();
}

void loop()
{

  IMU.readSensor();
  iteration_counter++;
  //unrotated IMU measurements
  float accelX_raw, accelY_raw, accelZ_raw;
  float gyroX_raw, gyroY_raw, gyroZ_raw;
  float magX_raw, magY_raw, magZ_raw;

  //rotate imu measurements
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;

  float phi, psi; //roll and yaw angle

  int32_t current_wheel_count = wheel_counter.read();
  float current_vel = getBikeSpeed(current_wheel_count);
  // The axis is rotated so that it matches the whipple model.
  accelY_raw = -IMU.getAccelX_mss();
  accelX_raw = IMU.getAccelY_mss();
  accelZ_raw = IMU.getAccelZ_mss();

  accelY = 0.9939 * accelX_raw - 0.0061 * accelY_raw - 0.1105 * accelZ_raw;
  accelX = 0.006069 * accelX_raw + 1 * accelY_raw - 0.000675 * accelZ_raw;
  accelZ = 0.1105 * accelX_raw + 0 * accelY_raw - 0.9939 * accelZ_raw;

  gyroY_raw = -IMU.getGyroX_rads();
  gyroX_raw = IMU.getGyroY_rads();
  gyroZ_raw = IMU.getGyroZ_rads();

  gyroX = 0.9939 * gyroX_raw - 0.0061 * gyroY_raw - 0.1105 * gyroZ_raw;
  gyroY = 0.006069 * gyroX_raw + 1 * gyroY_raw - 0.000675 * gyroZ_raw;
  gyroZ = 0.1105 * gyroX_raw + 0 * gyroY_raw - 0.9939 * gyroZ_raw;

  magY_raw = -IMU.getMagX_uT();
  magX_raw = IMU.getMagY_uT();
  magZ_raw = IMU.getMagZ_uT();

  magX = 0.9939 * magX_raw - 0.0061 * magY_raw - 0.1105 * magZ_raw;
  magY = 0.006069 * magX_raw + 1 * magY_raw - 0.000675 * magZ_raw;
  magZ = 0.1105 * magX_raw + 0 * magY_raw - 0.9939 * magZ_raw;

  // Compute pseudo-absolute measurement of roll from gyroscope.
  float phi_d = atan2f(gyroZ * current_vel, g);
  float phi_w = atan2f(gyroY, gyroZ);
  float W = expf(-pow(previous_phi_m, 2) / 0.05);
  float phi_m = W * phi_d + (1 - W) * phi_w;
  previous_phi_m = phi_m;
  t2 = micros();
  float dt = t2 - t1;
  t1 = t2;
  // at the start we compute the roll from the accelerometer readigs
  if (iteration_counter < 10000)
  {
    phi = atan2f(accelY, accelZ);
    roll_kalman.setAngle(phi);
    previous_phi = phi;
  }
  else if (sensorfuse)
  {
    a = 0.001;
    phi = (1 - a) * (previous_phi + gyroX * dt) + a * phi_m;
    previous_phi = phi;
  }
  else
  {
    phi = roll_kalman.getAngle(phi_m, gyroX, dt);
    previous_phi = phi;
  }
  //correct for eueler angle rotation sequence
  float yaw_rate = gyroY * sinf(phi) + gyroZ * cosf(phi);
  //estimation from magnetometer readings.
  float psi_mag = atan2f(magZ * sinf(phi) - magY * cosf(phi), magX);
  if (sensorfuse)
  {
    a = 0.00027;
    psi = (1 - a) * (previous_psi + yaw_rate * dt) + a * psi_mag;
    previous_psi = psi;
  }
  else
  {
    psi = yaw_kalman.getAngle(psi_mag, yaw_rate, dt);
    previous_psi = psi;
  }
}

float getBikeSpeed(int32_t current_wheel_count)
{
  static int32_t wheel_counts_index = 0;
  int32_t previous_wheel_count = wheel_counts[wheel_counts_index];
  wheel_counts[wheel_counts_index] = current_wheel_count;
  wheel_counts_index += 1;
  if (wheel_counts_index >= WHEEL_COUNTS_LENGTH)
  {
    wheel_counts_index = 0;
  }
  float rps_wheel = ((float)(current_wheel_count - previous_wheel_count)) / 192.0f /* counts/rev */ * 1000.0f /* ms / s */ / ((float)WHEEL_COUNTS_LENGTH);
  float bike_velocity_ms = -rps_wheel * 6.28f * 0.33f / 1000.0f * 3600.0f /* radius in m */ * 0.277778 /*km/h to m/s*/;

  return bike_velocity_ms;
}
