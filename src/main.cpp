#include "Wire.h"
#include "I2Cdev.h"
#include "ITG3200.h"
#include "ADXL345.h"
#include <QMC5883LCompass.h>

ITG3200 gyro;
ADXL345 accel;
QMC5883LCompass compass;

int16_t gxRaw, gyRaw, gzRaw; // Raw gyroscope readings
int16_t axRaw, ayRaw, azRaw; // Raw accelerometer readings
int16_t mxRaw, myRaw, mzRaw; // Raw magnetometer readings

// Mahony filter variables
float Kp = 2.0f; // Proportional gain (tuned for responsiveness)
float Ki = 0.05f; // Integral gain (tuned for drift correction)
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

// Quaternion and timing
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
unsigned long lastUpdate = 0;

void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void quaternionToEuler(float &roll, float &pitch, float &yaw);

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Initialize sensors
  gyro.initialize();
  accel.initialize();
  compass.init();
  
  // Set sensor ranges (important for unit conversion)
  accel.setRange(ADXL345_RANGE_4G);
  gyro.setFullScaleRange(ITG3200_FULLSCALE_2000);
  compass.setCalibrationOffsets(10.00, -49.00, 85.00);
  compass.setCalibrationScales(1.03, 0.90, 1.09);

  lastUpdate = millis();
}

void loop() {
  // Calculate actual time step
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0f;
  lastUpdate = now;

  // Read raw sensor data
  gyro.getRotation(&gxRaw, &gyRaw, &gzRaw);
  accel.getAcceleration(&axRaw, &ayRaw, &azRaw);
  compass.read();
  mxRaw = compass.getX();
  myRaw = compass.getY();
  mzRaw = compass.getZ();

  // Convert sensor data to proper units
  // Gyro: degrees/s -> radians/s
  float gx = gxRaw * (M_PI / 180.0f);
  float gy = gyRaw * (M_PI / 180.0f);
  float gz = gzRaw * (M_PI / 180.0f);

  // Accelerometer: raw -> m/s² (assuming 4G range)
  float ax = axRaw * 0.0039f * 9.81f;
  float ay = ayRaw * 0.0039f* 9.81f;
  float az = azRaw * 0.0039f * 9.81f;

  // Magnetometer: raw -> μT (calibrated)
  float mx = mxRaw * 0.15f; // QMC5883L sensitivity: 0.15 μT/LSB
  float my = myRaw * 0.15f;
  float mz = mzRaw * 0.15f;

  // Update Mahony filter
  mahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

  // Convert to Euler angles
  float roll, pitch, yaw;
  quaternionToEuler(roll, pitch, yaw);

  // Print results

  Serial.print("Orientation: ");
  Serial.print(roll); Serial.print(" ");
  Serial.print(pitch); Serial.print(" ");
  Serial.println(yaw);
  

  // Maintain ~100Hz update rate
  if (dt < 0.01) delay(10 - (millis() - now));
}

// Improved Mahony AHRS implementation
void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;

  // Normalize accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0) return;
  ax /= norm; ay /= norm; az /= norm;

  // Normalize magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0) return;
  mx /= norm; my /= norm; mz /= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * (mx * (0.5f - q2*q2 - q3*q3) + my * (q1*q2 - q0*q3) + mz * (q1*q3 + q0*q2));
  hy = 2.0f * (mx * (q1*q2 + q0*q3) + my * (0.5f - q1*q1 - q3*q3) + mz * (q2*q3 - q0*q1));
  hz = 2.0f * mx * (q1*q3 - q0*q2) + 2.0f * my * (q2*q3 + q0*q1) + 2.0f * mz * (0.5f - q1*q1 - q2*q2);
  bx = sqrt(hx*hx + hy*hy);
  bz = hz;

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q1*q3 - q0*q2);
  vy = 2.0f * (q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  wx = 2.0f * bx * (0.5f - q2*q2 - q3*q3) + 2.0f * bz * (q1*q3 - q0*q2);
  wy = 2.0f * bx * (q1*q2 - q0*q3) + 2.0f * bz * (q0*q1 + q2*q3);
  wz = 2.0f * bx * (q0*q2 + q1*q3) + 2.0f * bz * (0.5f - q1*q1 - q2*q2);

  // Error is cross product between estimated and measured directions
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  // Integrate error
  integralFBx += ex * Ki * dt;
  integralFBy += ey * Ki * dt;
  integralFBz += ez * Ki * dt;

  // Apply feedback to gyro
  gx += Kp * ex + integralFBx;
  gy += Kp * ey + integralFBy;
  gz += Kp * ez + integralFBz;

  // Integrate quaternion
  q0 += (-q1*gx - q2*gy - q3*gz) * 0.5f * dt;
  q1 += (q0*gx + q2*gz - q3*gy) * 0.5f * dt;
  q2 += (q0*gy - q1*gz + q3*gx) * 0.5f * dt;
  q3 += (q0*gz + q1*gy - q2*gx) * 0.5f * dt;

  // Normalize quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (norm == 0) return;
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
}

// Convert quaternion to Euler angles in degrees
void quaternionToEuler(float &roll, float &pitch, float &yaw) {
  roll = atan2(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * (180.0f / M_PI);
  pitch = asin(2.0f * (q0*q2 - q3*q1)) * (180.0f / M_PI);
  yaw = atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * (180.0f / M_PI);
  
  // Convert yaw to 0-360° compass heading
  if (yaw < 0) yaw += 360.0f;
}