#include "../../include/header.h"

#include <Arduino.h>
#include <SensorFusion.h>
#include <StateMachine.h>
#include <Telemetry.h>
#include <Buzzer.h>
#include <Led.h>

StateMachine stateMachine;
SensorFusion sensorFusion;
Telemetry telemetry;
// LoRaManager loraManager;

void setup()
{
  SerialUSB.begin(115200);
  setupLed();

  stateMachine.init();
  sensorFusion.init();
  // telemetry.init();
  // loraManager.init();
  // setupBuzzer();

  // startupSound();
  startupBlink();
}

void loop()
{

  sensorFusion.update();
  stateMachine.updateState(sensorFusion.getData());

  // Handle telemetry
  telemetry.logData(sensorFusion.getData(), stateMachine.getCurrentState());
  // loraManager.sendTelemetry(sensorFusion.getData(), stateMachine.getCurrentState());

  // // Handle incoming commands
  // loraManager.processIncoming();

  // // Handle SOS mode
  // if (stateMachine.getCurrentState() == ProbeState::GROUND)
  // {
  //   digitalWrite(LED_PIN, HIGH);
  //   tone(BUZZER_PIN, 1000);

  //   if (digitalRead(BUTTON_PIN) == LOW)
  //   {
  //     loraManager.sendSOS();
  //   }
  // }

  delay(10); // Adjust based on required sampling rate
}