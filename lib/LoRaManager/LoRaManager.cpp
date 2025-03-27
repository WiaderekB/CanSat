#include "LoRaManager.h"
#include "../../include/header.h"
#include <cstring>
#include <cstdint>

LoRaManager::LoRaManager(StateMachine &stateMachine, SensorFusion &sensorFusion)
    : stateMachine(stateMachine), sensorFusion(sensorFusion) {}

bool transmitStart = true;
bool transmitEnd = false;
void setFlag()
{
  transmitEnd = true;
}

bool LoRaManager::init()
{
  int state = radio.begin(LORA_FREQUENCY);
  if (state != RADIOLIB_ERR_NONE)
  {
    SerialUSB.println("LoRa initialization failed");
    return false;
  }
  else
  {
    SerialUSB.println("LoRa initialized");
  }

  radio.setBandwidth(250.0);
  radio.setSyncWord(LORA_SYNC_WORD);
  radio.setOutputPower(17);
  radio.setPacketSentAction(setFlag); // Changed to use a function pointer instead of a lambda

  byte startByte[1] = {0x00};
  radio.startTransmit(startByte, 1);

  return true;
}

void LoRaManager::sendTelemetry(const SensorData &data, const ProbeState &state)
{
  if (transmitEnd)
  {
    transmitEnd = false;
    transmitStart = false;
    lastTransmitted = millis();
    radio.finishTransmit();
  }

  if (millis() - lastTransmitted > LORA_TRANSMIT_INTERVAL && !transmitStart)
  {
    // Create a binary packet buffer
    uint8_t packet[64]; // Adjust size based on your data
    int index = 0;

    // Add timestamp (4 bytes)
    uint32_t timestamp = millis();
    packet[index++] = (timestamp >> 24) & 0xFF;
    packet[index++] = (timestamp >> 16) & 0xFF;
    packet[index++] = (timestamp >> 8) & 0xFF;
    packet[index++] = timestamp & 0xFF;

    // Add state (1 byte)
    packet[index++] = static_cast<uint8_t>(state);

    // Add GPS data (latitude, longitude, satellites, GPSValid)
    int32_t lat = static_cast<int32_t>(data.latitude * 1e6); // Convert to fixed-point
    int32_t lon = static_cast<int32_t>(data.longitude * 1e6);
    packet[index++] = (lat >> 24) & 0xFF;
    packet[index++] = (lat >> 16) & 0xFF;
    packet[index++] = (lat >> 8) & 0xFF;
    packet[index++] = lat & 0xFF;
    packet[index++] = (lon >> 24) & 0xFF;
    packet[index++] = (lon >> 16) & 0xFF;
    packet[index++] = (lon >> 8) & 0xFF;
    packet[index++] = lon & 0xFF;
    packet[index++] = data.satellites;
    packet[index++] = data.GPSValid ? 1 : 0;

    // Add altitude (4 bytes)
    int32_t alt = static_cast<int32_t>(data.altitude * 100); // Convert to cm
    packet[index++] = (alt >> 24) & 0xFF;
    packet[index++] = (alt >> 16) & 0xFF;
    packet[index++] = (alt >> 8) & 0xFF;
    packet[index++] = alt & 0xFF;

    // Add environmental data (temperature, pressure, humidity)
    int16_t temp = static_cast<int16_t>(data.temperature * 100); // Convert to 0.01°C
    int32_t press = static_cast<int32_t>(data.pressure * 100);   // Convert to Pa
    uint16_t hum = static_cast<uint16_t>(data.humidity * 100);   // Convert to 0.01%
    packet[index++] = (temp >> 8) & 0xFF;
    packet[index++] = temp & 0xFF;
    packet[index++] = (press >> 24) & 0xFF;
    packet[index++] = (press >> 16) & 0xFF;
    packet[index++] = (press >> 8) & 0xFF;
    packet[index++] = press & 0xFF;
    packet[index++] = (hum >> 8) & 0xFF;
    packet[index++] = hum & 0xFF;

    // Add orientation (3 floats -> 12 bytes)
    int16_t roll = static_cast<int16_t>(data.orientation[0] * 100); // Convert to 0.01°
    int16_t pitch = static_cast<int16_t>(data.orientation[1] * 100);
    int16_t yaw = static_cast<int16_t>(data.orientation[2] * 100);
    packet[index++] = (roll >> 8) & 0xFF;
    packet[index++] = roll & 0xFF;
    packet[index++] = (pitch >> 8) & 0xFF;
    packet[index++] = pitch & 0xFF;
    packet[index++] = (yaw >> 8) & 0xFF;
    packet[index++] = yaw & 0xFF;

    // Add acceleration (3 floats -> 12 bytes)
    int16_t accX = static_cast<int16_t>(data.acceleration[0] * 1000); // Convert to mg
    int16_t accY = static_cast<int16_t>(data.acceleration[1] * 1000);
    int16_t accZ = static_cast<int16_t>(data.acceleration[2] * 1000);
    packet[index++] = (accX >> 8) & 0xFF;
    packet[index++] = accX & 0xFF;
    packet[index++] = (accY >> 8) & 0xFF;
    packet[index++] = accY & 0xFF;
    packet[index++] = (accZ >> 8) & 0xFF;
    packet[index++] = accZ & 0xFF;

    // Transmit the packet
    transmitStart = true;
    radio.startTransmit(packet, index);
  }
}

void LoRaManager::processIncoming()
{
  uint8_t packet[64];
  int packetSize = radio.receive(packet, 64);

  if (packetSize > 0)
  {
    handleCommand(packet[0]); // First byte is the command
  }
}

void LoRaManager::handleCommand(uint8_t command)
{
  switch (command)
  {
  case 0x01: // Arm command
    if (stateMachine.getCurrentState() == ProbeState::STANDBY)
    {
      stateMachine.updateState(sensorFusion.getData());
    }
    break;
    // Add more commands as needed
  }
}

void LoRaManager::preparePacket(const SensorData &data, const ProbeState &state, uint8_t *packet, int &index)
{
  uint32_t timestamp = millis();
  int32_t lat = static_cast<int32_t>(data.latitude * 1e6);
  int32_t lon = static_cast<int32_t>(data.longitude * 1e6);
  int32_t alt = static_cast<int32_t>(data.altitude * 100);
  int16_t temp = static_cast<int16_t>(data.temperature * 100);
  int32_t press = static_cast<int32_t>(data.pressure * 100);
  uint16_t hum = static_cast<uint16_t>(data.humidity * 100);
  int16_t roll = static_cast<int16_t>(data.orientation[0] * 100);
  int16_t pitch = static_cast<int16_t>(data.orientation[1] * 100);
  int16_t yaw = static_cast<int16_t>(data.orientation[2] * 100);
  int16_t accX = static_cast<int16_t>(data.acceleration[0] * 1000);
  int16_t accY = static_cast<int16_t>(data.acceleration[1] * 1000);
  int16_t accZ = static_cast<int16_t>(data.acceleration[2] * 1000);

  auto appendData = [&](const void *src, size_t size)
  {
    memcpy(&packet[index], src, size);
    index += size;
  };

  appendData(&timestamp, sizeof(timestamp));
  appendData(&state, sizeof(state));
  appendData(&lat, sizeof(lat));
  appendData(&lon, sizeof(lon));
  appendData(&data.satellites, sizeof(data.satellites));
  uint8_t gpsValid = data.GPSValid ? 1 : 0;
  appendData(&gpsValid, sizeof(gpsValid));
  appendData(&alt, sizeof(alt));
  appendData(&temp, sizeof(temp));
  appendData(&press, sizeof(press));
  appendData(&hum, sizeof(hum));
  appendData(&roll, sizeof(roll));
  appendData(&pitch, sizeof(pitch));
  appendData(&yaw, sizeof(yaw));
  appendData(&accX, sizeof(accX));
  appendData(&accY, sizeof(accY));
  appendData(&accZ, sizeof(accZ));
}
