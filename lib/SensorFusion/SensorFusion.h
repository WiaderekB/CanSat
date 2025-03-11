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
  uint8_t satellites;
  bool GPSValid;
  bool BMPValid;
  bool BMEValid;
};

struct TempSensorData
{
  float temperatureBMP;
  float temperatureBME;
  float temperatureLM35;
  float pressureBMP;
  float pressureBME;
  float humidityBME;
  float altitudeBMP;
  float altitudeBME;
  float altitudeGPS;
  float longitudeGPS;
  float latitudeGPS;
  float accelerometr[3];
  float magnetometr[3];
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
  void readLM35();
  void readMPU6050();
  void readGPS();
  void fuseData();
};