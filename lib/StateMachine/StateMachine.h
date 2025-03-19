#pragma once
#include "SensorFusion.h"
#include "../../include/header.h"

enum class ProbeState
{
  STANDBY,
  ARMED,
  FLIGHT,
  APOGEE,
  DESCENT,
  GROUND
};

class StateMachine
{
public:
  void init();
  void updateState(const SensorData &data);
  const ProbeState &getCurrentState() const;

private:
  ProbeState currentState = ProbeState::STANDBY;
  unsigned long apogeeDetectionTime = 0;
  bool apogeeDetected = false;

  void detectLaunch(const SensorData &data);
  void detectApogee(const SensorData &data);
  void detectLanding(const SensorData &data);
};