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
#define ALPHA 0.6
float hardIronOffset[3] = {1.3080, 2.3450, -24.1910};
float softIronMatrix[3][3] = {{0.0678, 0.0000, 0.0000},
                              {0.0000, 0.0503, 0.0000},
                              {0.0000, 0.0000, 0.0465}};

int SensorFusion::init()
{
  Wire.begin();

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
    mpu.setXAccelOffset(1);
    mpu.setYAccelOffset(-1);
    mpu.setZAccelOffset(8);
    mpu.setXGyroOffset(1);
    mpu.setYGyroOffset(2);
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
  readMPU6050();
  readBMM150();
  readGPS();
  fuseData();
  calculateAcceleration();
  calculateVelocity();

  if (SerialUSB)
    printSensorData(finalData);
}

void SensorFusion::readBME280()
{
  finalData.temperature = bme.readTemperature();
  finalData.pressure = bme.readPressure() / 100;
  finalData.humidity = bme.readHumidity();
  tempData.altitudeBME = bme.readAltitude(1013.25);
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
  while (Serial.available() > 0)
  {
    char received = Serial.read();
    if (received == '\n')
    {
      processGPSData(gpsBuffer);
      gpsBuffer = ""; // Reset buffer after processing
    }
    else
    {
      gpsBuffer += received;
    }
  }
}

void SensorFusion::processGPSData(String nmeaSentence)
{
  if (nmeaSentence.startsWith("$GPGGA"))
  {
    SerialUSB.println("GGA Sentence Found: " + nmeaSentence);
    parseGGA(nmeaSentence);
  }
}

void SensorFusion::parseGGA(String ggaSentence)
{
  char *str = const_cast<char *>(ggaSentence.c_str());
  char *token = strtok(str, ",");
  int fieldIndex = 0;

  String lat, latDir, lon, lonDir, numSat, altitude;

  while (token != NULL)
  {
    switch (fieldIndex)
    {

    case 2:
      lat = token;
      break;
    case 3:
      latDir = token;
      break;
    case 4:
      lon = token;
      break;
    case 5:
      lonDir = token;
      break;
    case 7:
      numSat = token;
      break;
    case 9:
      altitude = token;
      break;
    }
    token = strtok(NULL, ",");
    fieldIndex++;
  }

  if (lat.length() > 0 && lon.length() > 0 && altitude.length() > 0)
  {
    finalData.GPSValid = true;
    lastGpsTime = millis();

    finalData.latitude = convertToDecimalDegrees(lat.toFloat(), latDir);
    finalData.longitude = convertToDecimalDegrees(lon.toFloat(), lonDir);
    tempData.altitudeGPS = altitude.toFloat();
    finalData.satellites = numSat.toInt();
  }
  else if (millis() - lastGpsTime > 750)
  {
    finalData.GPSValid = false;
  }
}

float SensorFusion::convertToDecimalDegrees(float nmeaCoord, String direction)
{
  int degrees = (int)(nmeaCoord / 100);
  float minutes = nmeaCoord - (degrees * 100);
  float decimalDegrees = degrees + (minutes / 60);
  if (direction == "S" || direction == "W")
  {
    decimalDegrees *= -1;
  }
  return decimalDegrees;
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
    finalData.altitude = tempData.altitudeGPS; // GPS altitude
  }

  finalData.rotationSpeed[0] = ALPHA * tempData.gyro[0] + (1 - ALPHA) * finalData.rotationSpeed[0];
  finalData.rotationSpeed[1] = ALPHA * tempData.gyro[1] + (1 - ALPHA) * finalData.rotationSpeed[1];
  finalData.rotationSpeed[2] = ALPHA * tempData.gyro[2] + (1 - ALPHA) * finalData.rotationSpeed[2];
}
void SensorFusion::calculateAcceleration()
{
  // Convert orientation angles from degrees to radians
  const float roll_rad = radians(finalData.orientation[0]);
  const float pitch_rad = radians(finalData.orientation[1]);
  const float yaw_rad = radians(finalData.orientation[2]);

  // Calculate trigonometric terms
  const float cr = cos(roll_rad);
  const float sr = sin(roll_rad);
  const float cp = cos(pitch_rad);
  const float sp = sin(pitch_rad);
  const float cy = cos(yaw_rad);
  const float sy = sin(yaw_rad);

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
  float az_world = R20 * ax + R21 * ay + R22 * az;

  // Subtract gravity (Z-component in world frame)
  az_world -= 9.81;

  finalData.acceleration[0] = ALPHA * ax_world + (1 - ALPHA) * finalData.acceleration[0];
  finalData.acceleration[1] = ALPHA * ay_world + (1 - ALPHA) * finalData.acceleration[1];
  finalData.acceleration[2] = ALPHA * az_world + (1 - ALPHA) * finalData.acceleration[2];
}

void SensorFusion::calculateVelocity()
{
  static uint32_t prevTime = millis();
  uint32_t currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0f; // Time in seconds
  prevTime = currentTime;

  // Integrate acceleration to get velocity (with drift mitigation)
  finalData.velocity[0] += finalData.acceleration[0] * dt;
  finalData.velocity[1] += finalData.acceleration[1] * dt;
  finalData.velocity[2] += finalData.acceleration[2] * dt;

  // Simple drift reduction: reset velocity if acceleration is near zero and device is stationary
  constexpr float ACCEL_THRESHOLD = 0.05f;
  constexpr float VELOCITY_THRESHOLD = 0.025f;
  if (fabs(finalData.acceleration[0]) < ACCEL_THRESHOLD &&
      fabs(finalData.acceleration[1]) < ACCEL_THRESHOLD &&
      fabs(finalData.acceleration[2]) < ACCEL_THRESHOLD)
  {
    // Decay velocity to zero if no significant acceleration
    finalData.velocity[0] *= 0.95;
    finalData.velocity[1] *= 0.95;
    finalData.velocity[2] *= 0.95;
  }

  // Prevent tiny residual velocities
  if (fabs(finalData.velocity[0]) < VELOCITY_THRESHOLD)
    finalData.velocity[0] = 0;
  if (fabs(finalData.velocity[1]) < VELOCITY_THRESHOLD)
    finalData.velocity[1] = 0;
  if (fabs(finalData.velocity[2]) < VELOCITY_THRESHOLD)
    finalData.velocity[2] = 0;
}

const SensorData &SensorFusion::getData() const
{
  return finalData;
}

void SensorFusion::printSensorData(SensorData sensorData)
{
  SerialUSB.println("--- Sensor Data ---");
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
