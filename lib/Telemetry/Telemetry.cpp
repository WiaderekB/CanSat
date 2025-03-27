#include "Telemetry.h"
#include "../../include/header.h"

#include <SPI.h>
#include <SD.h>
#include "StateMachine.h"
#include "SensorFusion.h"

bool Telemetry::init()
{
  if (!SD.begin(11))
  {
    SerialUSB.println("SD initialization failed");
    sdInitialized = false;
  }
  else
  {
    SerialUSB.println("SD initialized");
    sdInitialized = true;

    // Print header
    logFile = SD.open("datalog.csv", FILE_WRITE);
    if (logFile)
    {
      logFile.println("Timestamp,State,Latitude,Longitude,Satellites,GPSValid,Altitude,Temperature,Pressure,Humidity,OrientationX,OrientationY,OrientationZ,AccelerationX,AccelerationY,AccelerationZ,VelocityX,VelocityY,VelocityZ");
      logFile.close();
    }
  }

  return sdInitialized;
}

void Telemetry::logData(const SensorData &data, const ProbeState &state)
{
  if (!sdInitialized)
    return;

  logFile = SD.open("datalog.csv", FILE_WRITE);
  if (logFile)
  {
    logFile.println(formatData(data, state));
    logFile.close();
  }
}

String Telemetry::formatData(const SensorData &data, const ProbeState &state)
{
  String output;
  output += String(millis()) + ",";
  output += String(static_cast<int>(state)) + ",";
  output += String(data.latitude, 6) + ",";
  output += String(data.longitude, 6) + ",";
  output += String(data.satellites) + ",";
  output += String(data.GPSValid) + ",";
  output += String(data.altitude) + ",";
  output += String(data.temperature) + ",";
  output += String(data.pressure) + ",";
  output += String(data.humidity) + ",";
  output += String(data.orientation[0]) + ",";
  output += String(data.orientation[1]) + ",";
  output += String(data.orientation[2]) + ",";
  output += String(data.acceleration[0]) + ",";
  output += String(data.acceleration[1]) + ",";
  output += String(data.acceleration[2]) + ",";
  output += String(data.velocity[0]) + ",";
  output += String(data.velocity[1]) + ",";
  output += String(data.velocity[2]);

  return output;
}
