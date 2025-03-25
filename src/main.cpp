#include "../../include/header.h"

#include <Arduino.h>
#include <vector>
#include <SensorFusion.h>
#include <StateMachine.h>
#include <Telemetry.h>
#include <Buzzer.h>
#include <Led.h>
#include <LoRaManager.h>
#include <Button.h>

StateMachine stateMachine;
SensorFusion sensorFusion;
Telemetry telemetry;
LoRaManager loraManager(stateMachine, sensorFusion);
Button button(stateMachine);

uint32_t buzzerOffSince, buzzerOnSince, ledOnSince, ledOffSince = 0;
bool buzzerState, ledState = false;

void setup()
{
  SerialUSB.begin(115200);

  setupLed();
  startupBlink();

  bool stateMachineInit = stateMachine.init();
  setLed(stateMachineInit ? PINK : BLUE, {0});

  bool sensorFusionInit = sensorFusion.init();
  setLed(sensorFusionInit ? PINK : BLUE, {1});

  bool loraManagerInit = loraManager.init();
  setLed(loraManagerInit ? PINK : BLUE, {2});

  bool telemetryInit = telemetry.init();
  setLed(telemetryInit ? PINK : BLUE, {3});

  setLed(stateMachineInit && sensorFusionInit && loraManagerInit && telemetryInit ? GREEN : RED, {0, 1, 2, 3});
  setupBuzzer();

  startupSound();
  blinkDone();
}

void loop()
{
  button.checkButton(stateMachine.getCurrentState());

  // if (stateMachine.getCurrentState() != ProbeState::STANDBY)
  // {

  sensorFusion.update();
  stateMachine.updateState(sensorFusion.getData());

  // Handle telemetry
  loraManager.sendTelemetry(sensorFusion.getData(), stateMachine.getCurrentState());
  // telemetry.logData(sensorFusion.getData(), stateMachine.getCurrentState());

  // Handle incoming commands
  // loraManager.processIncoming();

  // Handle SOS mode
  if (stateMachine.getCurrentState() == ProbeState::GROUND)
  {

    if (ledState && millis() - ledOnSince > 250)
    {
      ledOffSince = millis();
      ledState = false;
      clearLed();
    }
    else if (!ledState && millis() - ledOffSince > 2000)
    {
      ledOnSince = millis();
      ledState = true;
      setLed(RED, {0, 1, 2, 3});
      setLed(RED, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, UP);
    }

    if (buzzerState && millis() - buzzerOnSince > 500)
    {
      buzzerOffSince = millis();
      buzzerState = false;
      buzzerOff();
    }
    else if (!buzzerState && millis() - buzzerOffSince > 4000)
    {
      buzzerOnSince = millis();
      buzzerState = true;
      buzzerOn();
    }
  }
  // }
}