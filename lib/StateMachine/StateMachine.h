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
  GROUND,
  SOS
};

class StateMachine
{
public:
  int init();
  void updateState(const SensorData &data);
  void setProbeState(ProbeState state);
  const ProbeState &getCurrentState() const;

private:
  ProbeState currentState = ProbeState::STANDBY;
  unsigned long apogeeDetectionTime = 0;
  bool apogeeDetected = false;

  void detectLaunch(const SensorData &data);
  void detectApogee(const SensorData &data);
  void detectLanding(const SensorData &data);
};