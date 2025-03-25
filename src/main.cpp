#include "../../include/header.h"

#include <Arduino.h>
#include <vector>
#include <SensorFusion.h>
#include <StateMachine.h>
#include <Telemetry.h>
#include <Buzzer.h>
#include <Led.h>
#include <LoRaManager.h>

StateMachine stateMachine;
SensorFusion sensorFusion;
Telemetry telemetry;
LoRaManager loraManager(stateMachine, sensorFusion);

void setup()
{
  SerialUSB.begin(115200);
  while (!SerialUSB)
    ;

  SerialUSB.println("Starting up...");
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
  sensorFusion.update();
  stateMachine.updateState(sensorFusion.getData());

  // Handle telemetry
  loraManager.sendTelemetry(sensorFusion.getData(), stateMachine.getCurrentState());
  // telemetry.logData(sensorFusion.getData(), stateMachine.getCurrentState());

  // Handle incoming commands
  // loraManager.processIncoming();

  // // Handle SOS mode
  // if (stateMachine.getCurrentState() == ProbeState::GROUND)
  // {
  // digitalWrite(LED_PIN, HIGH);
  // tone(BUZZER_PIN, 1000);

  // if (digitalRead(BUTTON_PIN) == LOW)
  // {
  //   loraManager.sendSOS();
  // }
  // }

  // delay(10); // Adjust based on required sampling rate
}