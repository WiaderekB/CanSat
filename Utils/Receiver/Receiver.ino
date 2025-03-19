
#include <RadioLib.h>

#define LORA_CS 10
#define LORA_DIO0 12
#define LORA_FREQUENCY 433.0 // 433 MHz
#define LORA_SYNC_WORD 0x12
#define PACKET_LEN 46 // Fixed packet size
#define LORA_TRANSMIT_INTERVAL 250

SX1278 radio = new Module(LORA_CS, LORA_DIO0, RADIOLIB_NC, RADIOLIB_NC);
volatile bool receivedFlag = false;

void setFlag(void) {
  receivedFlag = true;
}

void setup() {
  SerialUSB.begin(9600);
  while(!SerialUSB);

  // initialize SX1278 with default settings
  SerialUSB.print(F("[SX1278] Initializing ... "));
  int state = radio.begin(LORA_FREQUENCY);
  if (state == RADIOLIB_ERR_NONE) {
    SerialUSB.println(F("success!"));
  } else {
    SerialUSB.print(F("failed, code "));
    SerialUSB.println(state);
    while (true) { delay(10); }
  }

  radio.setBandwidth(250.0);
  radio.setSyncWord(LORA_SYNC_WORD);
  radio.setOutputPower(17);
  radio.setPacketSentAction(setFlag);

  SerialUSB.print(F("[SX1278] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    SerialUSB.println(F("success!"));
  } else {
    SerialUSB.print(F("failed, code "));
    SerialUSB.println(state);
    while (true) { delay(10); }
  }
}

void loop() {
  if(receivedFlag) {
    receivedFlag = false;   
    byte packet[64];
    int numBytes = radio.getPacketLength();
    int state = radio.readData(packet, numBytes);

    if (state == RADIOLIB_ERR_NONE) {
    int index = 0;

    // Decode timestamp (4 bytes)
    uint32_t timestamp = (packet[index++] << 24) | (packet[index++] << 16) | (packet[index++] << 8) | packet[index++];

    // Decode state (1 byte)
    uint8_t state = packet[index++];

    // Decode GPS data (latitude, longitude, satellites, GPSValid)
    int32_t lat = (packet[index++] << 24) | (packet[index++] << 16) | (packet[index++] << 8) | packet[index++];
    int32_t lon = (packet[index++] << 24) | (packet[index++] << 16) | (packet[index++] << 8) | packet[index++];
    uint8_t satellites = packet[index++];
    bool gpsValid = packet[index++] == 1;

    // Decode altitude (4 bytes)
    int32_t alt = (packet[index++] << 24) | (packet[index++] << 16) | (packet[index++] << 8) | packet[index++];

    // Decode environmental data (temperature, pressure, humidity)
    int16_t temp = (packet[index++] << 8) | packet[index++];
    int32_t press = (packet[index++] << 24) | (packet[index++] << 16) | (packet[index++] << 8) | packet[index++];
    uint16_t hum = (packet[index++] << 8) | packet[index++];

    // Decode orientation (roll, pitch, yaw)
    int16_t roll = (packet[index++] << 8) | packet[index++];
    int16_t pitch = (packet[index++] << 8) | packet[index++];
    int16_t yaw = (packet[index++] << 8) | packet[index++];

    // Decode acceleration (accX, accY, accZ)
    int16_t accX = (packet[index++] << 8) | packet[index++];
    int16_t accY = (packet[index++] << 8) | packet[index++];
    int16_t accZ = (packet[index++] << 8) | packet[index++];

    // Convert fixed-point values back to floats
    float latitude = lat / 1e6f;
    float longitude = lon / 1e6f;
    float altitude = alt / 100.0f;
    float temperature = temp / 100.0f;
    float pressure = press / 100.0f;
    float humidity = hum / 100.0f;
    float orientation[3] = {roll / 100.0f, pitch / 100.0f, yaw / 100.0f};
    float acceleration[3] = {accX / 1000.0f, accY / 1000.0f, accZ / 1000.0f};

    // Print decoded data
    SerialUSB.println("\n===== Received Data =====");
    SerialUSB.print("Timestamp: "); SerialUSB.println(timestamp);
    SerialUSB.print("State: "); SerialUSB.println(state);
    SerialUSB.print("Location: Lat "); SerialUSB.print(latitude, 6);
    SerialUSB.print(", Lon "); SerialUSB.println(longitude, 6);
    SerialUSB.print("Satellites: "); SerialUSB.println(satellites);
    SerialUSB.print("GPS Valid: "); SerialUSB.println(gpsValid ? "Yes" : "No");
    SerialUSB.print("Altitude: "); SerialUSB.print(altitude); SerialUSB.println(" m");
    SerialUSB.print("Temperature: "); SerialUSB.print(temperature); SerialUSB.println(" °C");
    SerialUSB.print("Pressure: "); SerialUSB.print(pressure); SerialUSB.println(" Pa");
    SerialUSB.print("Humidity: "); SerialUSB.print(humidity); SerialUSB.println(" %");
    SerialUSB.print("Orientation: Roll "); SerialUSB.print(orientation[0]);
    SerialUSB.print(", Pitch "); SerialUSB.print(orientation[1]);
    SerialUSB.print(", Yaw "); SerialUSB.println(orientation[2]);
    SerialUSB.print("Acceleration: X "); SerialUSB.print(acceleration[0]);
    SerialUSB.print(", Y "); SerialUSB.print(acceleration[1]);
    SerialUSB.print(", Z "); SerialUSB.println(acceleration[2]);
    SerialUSB.println("========================\n");

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      SerialUSB.println(F("[SX1278] CRC error!"));

    } else {
      // some other error occurred
      SerialUSB.print(F("[SX1278] Failed, code "));
      SerialUSB.println(state);

    }
  }
}
