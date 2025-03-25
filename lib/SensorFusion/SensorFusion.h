#pragma once
#include "../../include/header.h"
#include <stdint.h>
#include <Arduino.h>

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
  float altitudeBME;
  float altitudeGPS;
  float accelerometer[3];
  float magnetometer[3];
  float gyro[3];
};

class SensorFusion
{
public:
  int init();
  void update();
  const SensorData &getData() const;

private:
  SensorData finalData;
  TempSensorData tempData;
  String gpsBuffer;
  uint32_t lastGpsTime = 0;

  void readBME280();
  void readMPU6050();
  void readBMM150();
  void readGPS();
  void processGPSData(String nmeaSentence);
  void parseGGA(String ggaSentence);
  float convertToDecimalDegrees(float nmeaCoord, String direction);

  void fuseData();
  void calculateAcceleration();
  void calculateVelocity();
  void printSensorData(SensorData sensorData);
};