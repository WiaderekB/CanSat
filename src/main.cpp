#include "../../include/header.h"

#include <Arduino.h>
#include <SensorFusion.h>
#include <StateMachine.h>
#include <Telemetry.h>

StateMachine stateMachine;
SensorFusion sensorFusion;
Telemetry telemetry;
// LoRaManager loraManager;

void setup()
{
  SerialUSB.begin(115200);

  // while (!Serial)
  //   ;

  // pinMode(LED_PIN, OUTPUT);
  // pinMode(BUZZER_PIN, OUTPUT);
  // pinMode(BUTTON_PIN, INPUT_PULLUP);

  sensorFusion.init();
  // telemetry.init();
  // loraManager.init();
  stateMachine.init();
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

  delay(100); // Adjust based on required sampling rate
}