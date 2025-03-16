#pragma once
#include "../../include/header.h"
#include <stdint.h>

struct SensorData
{
  float temperature;
  float pressure;
  float humidity;
  float altitude;
  float longitude;
  float latitude;
  float orientation[3];
  float acceleration[3];
  float velocity[3];
  float rotationSpeed[3];
  bool GPSValid;
  uint8_t satellites;
};

struct TempSensorData
{
  float temperatureBMP;
  float temperatureBME;
  float pressureBMP;
  float pressureBME;
  float altitudeBMP;
  float altitudeBME;
  float altitudeGPS;
  float longitudeGPS;
  float latitudeGPS;
  float accelerometer[3];
  float magnetometer[3];
  float gyro[3];
};

class SensorFusion
{
public:
  void init();
  void update();
  const SensorData &getData() const;

private:
  SensorData finalData;
  TempSensorData tempData;
  void readBME280();
  void readBMP280();
  void readMPU6050();
  void readBMM150();
  void readGPS();
  void fuseData();
  void calculateAcceleration();
  void calculateVelocity();
  void printTempSensorData(TempSensorData tempData);
  void printSensorData(SensorData sensorData);
};