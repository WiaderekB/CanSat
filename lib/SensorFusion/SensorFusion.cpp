#include <Arduino.h>

#include "SensorFusion.h"
#include <MPU6050_light.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>

Adafruit_BME280 bme;
Adafruit_BMP280 bmp;
MPU6050 mpu(Wire);
TinyGPSPlus gps;

void SensorFusion::init()
{
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets(true, true); // gyro and accelero

  // GPS initialization
  // Serial.begin(9600); // GPS module on Serial
}

void SensorFusion::update()
{
  readBME280();
  readBMP280();
  readMPU6050();
  readGPS();
  fuseData();
}

void SensorFusion::readBME280()
{
  if (bme.begin(BME280_ADDRESS))
  {
    tempData.temperatureBME = bme.readTemperature();
    tempData.pressureBME = bme.readPressure();
    tempData.humidityBME = bme.readHumidity();
    tempData.altitudeBME = bme.readAltitude(1013.25);
    finalData.BMEValid = true;
  }
  else
  {
    finalData.BMEValid = false;
  }
}
void SensorFusion::readBMP280()
{
  if (bmp.begin(BMP280_ADDRESS))
  {
    tempData.temperatureBMP = bmp.readTemperature();
    tempData.pressureBMP = bmp.readPressure();
    tempData.altitudeBMP = bmp.readAltitude(1013.25);
    finalData.BMPValid = true;
  }
  else
  {
    finalData.BMPValid = false;
  }
}

void SensorFusion::readLM35()
{
  int raw = analogRead(LM35);
  float voltage = raw * 3.3 / (std::pow(2, 12));
  tempData.temperatureLM35 = 100.0 * voltage;
}

void SensorFusion::readMPU6050()
{
  mpu.update();
  tempData.accelerometr[0] = mpu.getAccX();
  tempData.accelerometr[1] = mpu.getAccY();
  tempData.accelerometr[2] = mpu.getAccZ();
  tempData.gyro[0] = mpu.getGyroX();
  tempData.gyro[1] = mpu.getGyroY();
  tempData.gyro[2] = mpu.getGyroZ();
}

void SensorFusion::readGPS()
{
  finalData.GPSValid = false;
  // while (Serial.available())
  // {
  //   gps.encode(Serial.read());
  // }

  // if (gps.location.isUpdated())
  // {
  //   tempData.gpsLat = gps.location.lat();
  //   tempData.gpsLon = gps.location.lng();
  //   tempData.gpsAlt = gps.altitude.meters();
  //   tempData.satellites = gps.satellites.value();
  //   tempData.gpsValid = true;
  // }
}

void SensorFusion::fuseData()
{
  finalData.temperature = (tempData.temperatureBME + tempData.temperatureBMP + tempData.temperatureLM35) / 3;
  finalData.pressure = (tempData.pressureBME + tempData.pressureBMP) / 2;
  finalData.humidity = tempData.humidityBME;
  finalData.altitude = (tempData.altitudeBME + tempData.altitudeBMP) / 2;
  finalData.acceleration[0] = tempData.accelerometr[0];
  finalData.acceleration[1] = tempData.accelerometr[1];
  finalData.acceleration[2] = tempData.accelerometr[2];
  finalData.orientation[0] = tempData.gyro[0];
  finalData.orientation[1] = tempData.gyro[1];
  finalData.orientation[2] = tempData.gyro[2];
}

const SensorData &SensorFusion::getData() const
{
  return finalData;
}