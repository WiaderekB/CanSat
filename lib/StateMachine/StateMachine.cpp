#include "StateMachine.h"
#include <Arduino.h>

bool StateMachine::init()
{
  currentState = ProbeState::STANDBY;
  return true;
}

void StateMachine::updateState(const SensorData &data)
{
  switch (currentState)
  {
  case ProbeState::ARMED:
    detectLaunch(data);
    break;

  case ProbeState::FLIGHT:
    detectApogee(data);
    break;

  case ProbeState::APOGEE:
    if (millis() - apogeeDetectionTime > APOGEE_DELAY_MS)
    {
      currentState = ProbeState::DESCENT;
    }
    break;

  case ProbeState::DESCENT:
    detectLanding(data);
    break;
  }
}

void StateMachine::detectLaunch(const SensorData &data)
{
  float totalAccel = sqrt(sq(data.acceleration[0]) +
                          sq(data.acceleration[1]) +
                          sq(data.acceleration[2]));
  if (totalAccel > LAUNCH_ACCEL_THRESHOLD)
  {
    currentState = ProbeState::FLIGHT;
  }
}

void StateMachine::detectApogee(const SensorData &data)
{
  static float prevAltitude = data.altitude;
  if (data.altitude < prevAltitude && !apogeeDetected)
  {
    apogeeDetectionTime = millis();
    apogeeDetected = true;
  }
  prevAltitude = data.altitude;

  if (apogeeDetected && (millis() - apogeeDetectionTime > APOGEE_DELAY_MS))
  {
    currentState = ProbeState::APOGEE;
  }
}

void StateMachine::detectLanding(const SensorData &data)
{
  static unsigned long stableStart = 0;
  float totalAccel = sqrt(sq(data.acceleration[0]) +
                          sq(data.acceleration[1]) +
                          sq(data.acceleration[2]));

  if (abs(totalAccel - 1.0) < LANDING_ACCEL_THRESHOLD)
  {
    if (stableStart == 0)
      stableStart = millis();
    if (millis() - stableStart > 5000)
    {
      currentState = ProbeState::GROUND;
    }
  }
  else
  {
    stableStart = 0;
  }
}

const ProbeState &StateMachine::getCurrentState() const
{
  return currentState;
}

void StateMachine::setProbeState(ProbeState state)
{
  currentState = state;
}