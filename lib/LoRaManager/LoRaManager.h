#pragma once
#include <RadioLib.h>
#include "../../include/header.h"
#include "StateMachine.h"
#include "SensorFusion.h"

class LoRaManager
{
public:
  LoRaManager(StateMachine &stateMachine, SensorFusion &sensorFusion);
  bool init();
  void sendTelemetry(const SensorData &data, const ProbeState &state);
  void processIncoming();

private:
  SX1278 radio = new Module(LORA_CS, LORA_DIO0, RADIOLIB_NC, RADIOLIB_NC);
  StateMachine &stateMachine;
  SensorFusion &sensorFusion;
  uint16_t lastTransmitted = 0;

  void handleCommand(uint8_t command);
  void sendSOS();
  void preparePacket(const SensorData &data, const ProbeState &state, uint8_t *packet, int &index);
};
