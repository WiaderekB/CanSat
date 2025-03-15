#pragma once

// Pin definitions
const int LM35 = 13; // LM35 A1
const int RGB = 9;   // RGB D3

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