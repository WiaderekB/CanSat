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
float hardIronOffset[3] = {1.3080, 2.3450, -24.1910};
float softIronMatrix[3][3] = {{0.0678, 0.0000, 0.0000},
                              {0.0000, 0.0503, 0.0000},
                              {0.0000, 0.0000, 0.0465}};

int SensorFusion::init()
{
  Wire.begin();
  // if (bmp.begin())
  // {
  //   SerialUSB.println("BMP280 initialized");
  // }
  // else
  // {
  //   SerialUSB.println("BMP280 initialization failed");
  // }
  if (bme.begin())
  {
    SerialUSB.println("BME280 initialized");
  }
  else
  {
    SerialUSB.println("BME280 initialization failed");
    return 1;
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
    return 1;
  }

  if (bmm150.begin() == BMM150_OK)
  {
    SerialUSB.println("BMM150 initialized");
    bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    bmm150.setRate(BMM150_DATA_RATE_10HZ);
    bmm150.setMeasurementXYZ();
  }
  else
  {
    SerialUSB.println("BMM150 initialization failed");
    return 1;
  }

  filter.begin();

  // GPS initialization
  Serial.begin(9600);
  if (Serial)
  {
    SerialUSB.println("GPS initialized");
  }
  else
  {
    SerialUSB.println("GPS initialization failed");
  }

  return 0;
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

  // printTempSensorData(tempData);
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

    SerialUSB.print(received);
    finalData.GPSValid = true;
  }
  else
  {
    finalData.GPSValid = false;
  }
}

void SensorFusion::fuseData()
{
  filter.update(tempData.gyro[0], tempData.gyro[1], tempData.gyro[2],
                tempData.accelerometer[0], tempData.accelerometer[1], tempData.accelerometer[2],
                tempData.magnetometer[0], tempData.magnetometer[1], tempData.magnetometer[2]);

  finalData.orientation[0] = filter.getRoll();
  finalData.orientation[1] = filter.getPitch();
  finalData.orientation[2] = filter.getYaw();

  finalData.altitude = tempData.altitudeBME;
  if (finalData.GPSValid)
  {
    finalData.altitude = (finalData.altitude + tempData.altitudeGPS) / 2; // GPS altitude
  }

  finalData.rotationSpeed[0] = tempData.gyro[0]; // Rotation around X-axis
  finalData.rotationSpeed[1] = tempData.gyro[1]; // Rotation around Y-axis
  finalData.rotationSpeed[2] = tempData.gyro[2]; // Rotation around Z-axis

  finalData.temperature = tempData.temperatureBME;
  finalData.pressure = tempData.pressureBME;
  finalData.latitude = tempData.latitudeGPS;
  finalData.longitude = tempData.longitudeGPS;
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

  // Get raw accelerometer data (assuming m/s²)
  const float ax = tempData.accelerometer[0];
  const float ay = tempData.accelerometer[1];
  const float az = tempData.accelerometer[2];

  // Construct rotation matrix (ZYX order)
  float R00 = cy * cp;
  float R01 = cy * sp * sr - sy * cr;
  float R02 = cy * sp * cr + sy * sr;

  float R10 = sy * cp;
  float R11 = sy * sp * sr + cy * cr;
  float R12 = sy * sp * cr - cy * sr;

  float R20 = -sp;
  float R21 = cp * sr;
  float R22 = cp * cr;

  // Apply rotation to accelerometer data
  float ax_world = R00 * ax + R01 * ay + R02 * az;
  float ay_world = R10 * ax + R11 * ay + R12 * az;
  float az_world = R20 * ax + R21 * ay + R22 * az - 9.81;

  // Apply complementary filter to reduce noise
  constexpr float FILTER_GAIN = 0.15f;
  finalData.acceleration[0] = FILTER_GAIN * ax_world + (1 - FILTER_GAIN) * finalData.acceleration[0];
  finalData.acceleration[1] = FILTER_GAIN * ay_world + (1 - FILTER_GAIN) * finalData.acceleration[1];
  finalData.acceleration[2] = FILTER_GAIN * az_world + (1 - FILTER_GAIN) * finalData.acceleration[2];
}

void SensorFusion::calculateVelocity()
{
  // Simple velocity integration (no damping)
  static float prevTime = 0;
  float dt = (millis() - prevTime) / 1000.0f; // Time in seconds
  prevTime = millis();

  finalData.velocity[0] += finalData.acceleration[0] * dt;
  finalData.velocity[1] += finalData.acceleration[1] * dt;
  finalData.velocity[2] += finalData.acceleration[2] * dt;
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
  SerialUSB.print(tempData.altitudeBME);
  SerialUSB.print(" m, ");

  SerialUSB.print("Humidity: ");
  SerialUSB.print(finalData.humidity);
  SerialUSB.println(" %");

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