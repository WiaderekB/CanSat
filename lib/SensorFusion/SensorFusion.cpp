#include <Arduino.h>
#include "SensorFusion.h"
#include <MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include "DFRobot_BMM150.h"
#include <Adafruit_AHRS.h>

Adafruit_NXPSensorFusion filter;
Adafruit_BME280 bme;
Adafruit_BMP280 bmp;
MPU6050 mpu;
TinyGPSPlus gps;
DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);

// Sensor calibration
#define ALPHA 0.75
float hardIronOffset[3] = {3.6800, 4.8770, -79.2420};
float softIronMatrix[3][3] = {{0.0578, 0.0000, 0.0000},
                              {0.0000, 0.0503, 0.0000},
                              {0.0000, 0.0000, 0.0465}};

void SensorFusion::init()
{
  Wire.begin();
  if (bmp.begin(BMP280_ADD))
  {
    SerialUSB.println("BMP280 initialized");
  }
  else
  {
    SerialUSB.println("BMP280 initialization failed");
  }
  if (bme.begin(BME280_ADD))
  {
    SerialUSB.println("BME280 initialized");
  }
  else
  {
    SerialUSB.println("BME280 initialization failed");
  }
  mpu.initialize();
  if (mpu.testConnection())
  {
    SerialUSB.println("MPU6050 initialized");
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.CalibrateAccel(7);
    mpu.CalibrateGyro(7);
  }
  else
  {
    SerialUSB.println("MPU6050 initialization failed");
  }

  bmm150.begin();
  if (bmm150.getDataReadyState())
  {
    SerialUSB.println("BMM150 initialized");
    bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    bmm150.setMeasurementXYZ();
  }
  else
  {
    SerialUSB.println("BMM150 initialization failed");
  }

  filter.begin();

  // GPS initialization
  Serial1.begin(9600); // GPS module on Serial
}

void SensorFusion::update()
{
  readBME280();
  readBMP280();
  readMPU6050();
  readBMM150();
  readGPS();
  fuseData();
  calculateAcceleration();
  calculateVelocity();

  printTempSensorData(tempData);
  // printSensorData(finalData);
}

void SensorFusion::readBME280()
{
  tempData.temperatureBME = bme.readTemperature();
  tempData.pressureBME = bme.readPressure() / 100;
  finalData.humidity = bme.readHumidity();
  tempData.altitudeBME = bme.readAltitude(1013.25);
}
void SensorFusion::readBMP280()
{
  tempData.temperatureBMP = bmp.readTemperature();
  tempData.pressureBMP = bmp.readPressure() / 100;
  tempData.altitudeBMP = bmp.readAltitude(1013.25);
}

void SensorFusion::readMPU6050()
{
  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

  float accelScale = 9.81 / 16384.0;
  float gyroScale = 1.0 / 131.0;

  float ax = rawAx * accelScale;
  float ay = rawAy * accelScale;
  float az = rawAz * accelScale;
  float gx = rawGx * gyroScale;
  float gy = rawGy * gyroScale;
  float gz = rawGz * gyroScale;

  // Apply a simple low-pass filter to reduce noise
  tempData.accelerometer[0] = ALPHA * ax + (1 - ALPHA) * tempData.accelerometer[0];
  tempData.accelerometer[1] = ALPHA * ay + (1 - ALPHA) * tempData.accelerometer[1];
  tempData.accelerometer[2] = ALPHA * az + (1 - ALPHA) * tempData.accelerometer[2];

  tempData.gyro[0] = ALPHA * gx + (1 - ALPHA) * tempData.gyro[0];
  tempData.gyro[1] = ALPHA * gy + (1 - ALPHA) * tempData.gyro[1];
  tempData.gyro[2] = ALPHA * gz + (1 - ALPHA) * tempData.gyro[2];
}

void SensorFusion::readBMM150()
{
  sBmm150MagData_t magData = bmm150.getGeomagneticData();

  // Apply hard-iron correction
  float x = magData.x - hardIronOffset[0];
  float y = magData.y - hardIronOffset[1];
  float z = magData.z - hardIronOffset[2];

  // Apply soft-iron correction
  float mx = x * softIronMatrix[0][0] + y * softIronMatrix[0][1] + z * softIronMatrix[0][2];
  float my = x * softIronMatrix[1][0] + y * softIronMatrix[1][1] + z * softIronMatrix[1][2];
  float mz = x * softIronMatrix[2][0] + y * softIronMatrix[2][1] + z * softIronMatrix[2][2];

  tempData.magnetometer[0] = ALPHA * mx + (1 - ALPHA) * tempData.magnetometer[0];
  tempData.magnetometer[1] = ALPHA * my + (1 - ALPHA) * tempData.magnetometer[1];
  tempData.magnetometer[2] = ALPHA * mz + (1 - ALPHA) * tempData.magnetometer[2];
}

void SensorFusion::readGPS()
{

  if (Serial.available())
  {
    char received = Serial.read();

    // SerialUSB.print(received);
    finalData.GPSValid = true;
  }
  else
  {
    finalData.GPSValid = false;
  }
}

void SensorFusion::fuseData()
{
  float roll, pitch, yaw;

  filter.update(tempData.gyro[0], tempData.gyro[1], tempData.gyro[2],
                tempData.accelerometer[0], tempData.accelerometer[1], tempData.accelerometer[2],
                tempData.magnetometer[0], tempData.magnetometer[1], tempData.magnetometer[2]);

  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();

  finalData.orientation[0] = roll;  // Roll (rad)
  finalData.orientation[1] = pitch; // Pitch (rad)
  finalData.orientation[2] = yaw;   // Yaw (rad)

  finalData.altitude = (tempData.altitudeBME + tempData.altitudeBMP) / 2;
  if (finalData.GPSValid)
  {
    finalData.altitude = (finalData.altitude + tempData.altitudeGPS) / 2; // GPS altitude
  }

  finalData.rotationSpeed[0] = tempData.gyro[0]; // Rotation around X-axis
  finalData.rotationSpeed[1] = tempData.gyro[1]; // Rotation around Y-axis
  finalData.rotationSpeed[2] = tempData.gyro[2]; // Rotation around Z-axis
}

void SensorFusion::calculateAcceleration()
{
  float R[9];
  const float roll = finalData.orientation[0];
  const float pitch = finalData.orientation[1];
  const float yaw = finalData.orientation[2];

  // Calculate trigonometric terms
  const float cr = cos(roll);
  const float sr = sin(roll);
  const float cp = cos(pitch);
  const float sp = sin(pitch);
  const float cy = cos(yaw);
  const float sy = sin(yaw);

  // Correct rotation matrix (body to world frame)
  // ZYX Tait-Bryan angles (yaw, pitch, roll)
  R[0] = cy * cp;
  R[1] = cy * sp * sr - sy * cr;
  R[2] = cy * sp * cr + sy * sr;

  R[3] = sy * cp;
  R[4] = sy * sp * sr + cy * cr;
  R[5] = sy * sp * cr - cy * sr;

  R[6] = -sp;
  R[7] = cp * sr;
  R[8] = cp * cr;

  // Get raw accelerometer data (assuming m/s²)
  const float ax = tempData.accelerometer[0];
  const float ay = tempData.accelerometer[1];
  const float az = tempData.accelerometer[2];

  // Transform to world frame (NED coordinates)
  float world_acc_x = R[0] * ax + R[1] * ay + R[2] * az;
  float world_acc_y = R[3] * ax + R[4] * ay + R[5] * az;
  float world_acc_z = R[6] * ax + R[7] * ay + R[8] * az;

  // Remove gravity (NED frame has +Z downward)
  constexpr float GRAVITY = 9.80665f;
  world_acc_z -= GRAVITY;

  // Apply complementary filter to reduce noise
  constexpr float FILTER_GAIN = 0.15f;
  finalData.acceleration[0] = FILTER_GAIN * world_acc_x + (1 - FILTER_GAIN) * finalData.acceleration[0];
  finalData.acceleration[1] = FILTER_GAIN * world_acc_y + (1 - FILTER_GAIN) * finalData.acceleration[1];
  finalData.acceleration[2] = FILTER_GAIN * world_acc_z + (1 - FILTER_GAIN) * finalData.acceleration[2];
}

void SensorFusion::calculateVelocity()
{
  // Simple velocity integration (no damping)
  static float prevTime = 0;
  float dt = (millis() - prevTime) / 1000.0f; // Time in seconds
  prevTime = millis();

  finalData.velocity[0] += tempData.accelerometer[0] * dt;
  finalData.velocity[1] += tempData.accelerometer[1] * dt;
  finalData.velocity[2] += tempData.accelerometer[2] * dt;
}

const SensorData &SensorFusion::getData() const
{
  return finalData;
}

void SensorFusion::printSensorData(SensorData sensorData)
{
  SerialUSB.println("--- Sensor Data ---");

  // Print SensorData fields
  SerialUSB.print("Temperature: ");
  SerialUSB.print(sensorData.temperature);
  SerialUSB.print(" °C, ");

  SerialUSB.print("Pressure: ");
  SerialUSB.print(sensorData.pressure);
  SerialUSB.print(" hPa, ");

  SerialUSB.print("Humidity: ");
  SerialUSB.print(sensorData.humidity);
  SerialUSB.println(" %");

  SerialUSB.print("Longitude: ");
  SerialUSB.print(sensorData.longitude);
  SerialUSB.print(" °, ");

  SerialUSB.print("Latitude: ");
  SerialUSB.print(sensorData.latitude);
  SerialUSB.print(" °, ");

  SerialUSB.print("Altitude: ");
  SerialUSB.print(sensorData.altitude);
  SerialUSB.println(" m");

  SerialUSB.print("Orientation - X: ");
  SerialUSB.print(sensorData.orientation[0]);
  SerialUSB.print(", Y: ");
  SerialUSB.print(sensorData.orientation[1]);
  SerialUSB.print(", Z: ");
  SerialUSB.println(sensorData.orientation[2]);

  SerialUSB.print("Acceleration - X: ");
  SerialUSB.print(sensorData.acceleration[0]);
  SerialUSB.print(", Y: ");
  SerialUSB.print(sensorData.acceleration[1]);
  SerialUSB.print(", Z: ");
  SerialUSB.println(sensorData.acceleration[2]);

  SerialUSB.print("Velocity - X: ");
  SerialUSB.print(sensorData.velocity[0]);
  SerialUSB.print(", Y: ");
  SerialUSB.print(sensorData.velocity[1]);
  SerialUSB.print(", Z: ");
  SerialUSB.println(sensorData.velocity[2]);

  SerialUSB.print("Rotation Speed - X: ");
  SerialUSB.print(sensorData.rotationSpeed[0]);
  SerialUSB.print(", Y: ");
  SerialUSB.print(sensorData.rotationSpeed[1]);
  SerialUSB.print(", Z: ");
  SerialUSB.println(sensorData.rotationSpeed[2]);

  SerialUSB.print("GPS Valid: ");
  SerialUSB.println(sensorData.GPSValid ? "True" : "False");

  SerialUSB.print("Satellites: ");
  SerialUSB.println(sensorData.satellites);

  SerialUSB.println("--------------------------");
}

void SensorFusion::printTempSensorData(TempSensorData tempData)
{
  SerialUSB.println("--- Raw Temp Sensor Data ---");

  // Print TempSensorData fields
  SerialUSB.print("BME280 - Temp: ");
  SerialUSB.print(tempData.temperatureBME);
  SerialUSB.print(" °C, ");

  SerialUSB.print("Pressure: ");
  SerialUSB.print(tempData.pressureBME);
  SerialUSB.print(" hPa, ");

  SerialUSB.print("Altitude: ");
  SerialUSB.println(tempData.altitudeBME);

  SerialUSB.print("BMP280 - Temp: ");
  SerialUSB.print(tempData.temperatureBMP);
  SerialUSB.print(" °C, ");

  SerialUSB.print("Pressure: ");
  SerialUSB.print(tempData.pressureBMP);
  SerialUSB.print(" hPa, ");

  SerialUSB.print("Altitude: ");
  SerialUSB.println(tempData.altitudeBMP);

  SerialUSB.print("BMM150 - Mag X: ");
  SerialUSB.print(tempData.magnetometer[0]);
  SerialUSB.print(" µT, Y: ");
  SerialUSB.print(tempData.magnetometer[1]);
  SerialUSB.print(" µT, Z: ");
  SerialUSB.println(tempData.magnetometer[2]);

  SerialUSB.print("MPU6050 - Accel X: ");
  SerialUSB.print(tempData.accelerometer[0]);
  SerialUSB.print(" m/s², Y: ");
  SerialUSB.print(tempData.accelerometer[1]);
  SerialUSB.print(" m/s², Z: ");
  SerialUSB.println(tempData.accelerometer[2]);

  SerialUSB.print("MPU6050 - Gyro X: ");
  SerialUSB.print(tempData.gyro[0]);
  SerialUSB.print(" °/s, Y: ");
  SerialUSB.print(tempData.gyro[1]);
  SerialUSB.print(" °/s, Z: ");
  SerialUSB.println(tempData.gyro[2]);

  SerialUSB.println("--------------------------");
}