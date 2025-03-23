void setup() {
    SerialUSB.begin(115200);    // Start SerialUSB for console communication
    Serial.begin(9600);         // GPS modules usually work at 9600 baud
    SerialUSB.println("Initializing GPS communication...");

    while (!SerialUSB) {
        // Wait for SerialUSB to be ready
    }
    SerialUSB.println("SerialUSB ready");
    SerialUSB.println("Serial initialized for GPS communication");
}

void loop() {
    if (Serial.available()) {  // Check if GPS data is received
        String gpsData = "";
        while (Serial.available()) {
            char received = Serial.read();
            gpsData += received;  // Append character to string
            if (received == '\n') break; // End of NMEA sentence
        }
        SerialUSB.print("GPS Data: ");
        SerialUSB.println(gpsData);  // Print full NMEA sentence
    }
}