#pragma once
#include <SD.h>
#include "../../include/header.h"
#include "SensorFusion.h"
#include "StateMachine.h"

class Telemetry
{
public:
  bool init();
  void logData(const SensorData &data, ProbeState state);

private:
  File logFile;
  bool sdInitialized = false;

  String formatData(const SensorData &data, ProbeState state);
};