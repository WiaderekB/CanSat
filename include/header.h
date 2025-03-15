#pragma once

// Pin definitions
const int BUZZER_PIN = 4; // BUZZER_PIN D4
const int RGB_PIN = 2;    // RGB D2
const int RGB_COUNT = 12;  // Number of RGB LEDs

// LoRa configuration
const long LORA_FREQUENCY = 433;
const int LORA_SYNC_WORD = 0x12;

// Sensor addresses
#define BME280_ADD 0x77
#define BMP280_ADD 0x76
#define MPU6050_ADDRESS 0x68

// Thresholds
const float LAUNCH_ACCEL_THRESHOLD = 1.5;  // g
const float LANDING_ACCEL_THRESHOLD = 0.2; // g
const int APOGEE_DELAY_MS = 2000;