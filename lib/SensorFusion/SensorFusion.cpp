#include <Arduino.h>
#include "SensorFusion.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>

Adafruit_BME280 bme;
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;

void SensorFusion::init()
{
  Wire.begin();
  mpu.begin();
  bmp.begin(BMP280_ADD);
  bme.begin(BME280_ADD);
  // GPS initialization
  Serial.begin(9600); // GPS module on Serial
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
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  tempData.accelerometr[0] = a.acceleration.x;
  tempData.accelerometr[1] = a.acceleration.y;
  tempData.accelerometr[2] = a.acceleration.z;

  tempData.gyro[0] = g.gyro.x;
  tempData.gyro[1] = g.gyro.y;
  tempData.gyro[2] = g.gyro.z;
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
  SerialUSB.println("--- Raw Sensor Data ---");
  SerialUSB.print("BME280 - Temp: ");
  SerialUSB.print(tempData.temperatureBME);
  SerialUSB.print(" °C, ");
  SerialUSB.print("Pressure: ");
  SerialUSB.print(tempData.pressureBME);
  SerialUSB.print(" hPa, ");
  SerialUSB.print("Humidity: ");
  SerialUSB.print(finalData.humidity);
  SerialUSB.print(" %, ");
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

  SerialUSB.print("MPU6050 - Accel X: ");
  SerialUSB.print(tempData.accelerometr[0]);
  SerialUSB.print(" m/s², Y: ");
  SerialUSB.print(tempData.accelerometr[1]);
  SerialUSB.print(" m/s², Z: ");
  SerialUSB.println(tempData.accelerometr[2]);

  SerialUSB.print("MPU6050 - Gyro X: ");
  SerialUSB.print(tempData.gyro[0]);
  SerialUSB.print(" °/s, Y: ");
  SerialUSB.print(tempData.gyro[1]);
  SerialUSB.print(" °/s, Z: ");
  SerialUSB.println(tempData.gyro[2]);
  SerialUSB.println("--------------------------");

  finalData.temperature = (tempData.temperatureBME + tempData.temperatureBMP) / 2;
  finalData.pressure = (tempData.pressureBME + tempData.pressureBMP) / 2;
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