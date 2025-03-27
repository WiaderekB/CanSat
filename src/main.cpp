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
  setupBuzzer();
  startupBlink();

  bool stateMachineInit = stateMachine.init();
  setLed(stateMachineInit ? BLUE : MAGENTA, {0});
  setLed(stateMachineInit ? BLUE : MAGENTA, {0, 1, 2}, DOWN);

  bool sensorFusionInit = sensorFusion.init();
  setLed(sensorFusionInit ? BLUE : MAGENTA, {1});
  setLed(sensorFusionInit ? BLUE : MAGENTA, {3, 4, 5}, DOWN);

  bool loraManagerInit = loraManager.init();
  setLed(loraManagerInit ? BLUE : MAGENTA, {2});
  setLed(loraManagerInit ? BLUE : MAGENTA, {6, 7, 8}, DOWN);

  bool telemetryInit = telemetry.init();
  setLed(telemetryInit ? BLUE : MAGENTA, {3});
  setLed(telemetryInit ? BLUE : MAGENTA, {9, 10, 11}, DOWN);

  bool successInit = stateMachineInit && sensorFusionInit && loraManagerInit && telemetryInit;
  blinkDone(successInit);
  startupSound(successInit);
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
  telemetry.logData(sensorFusion.getData(), stateMachine.getCurrentState());

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