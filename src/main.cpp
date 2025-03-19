#include "../../include/header.h"

#include <Arduino.h>
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
  // while (!SerialUSB)
  // ;

  SerialUSB.println("Starting up...");
  setupLed();
  startupBlink();

  stateMachine.init();
  sensorFusion.init();
  loraManager.init();
  // telemetry.init();
  // setupBuzzer();

  // startupSound();
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