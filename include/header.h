#pragma once

// Pin definitions
const int BUZZER_PIN = 7; // BUZZER_PIN D7

const int RGB_DOWN_PIN = 2;    // RGB D2
const int RGB_DOWN_COUNT = 12; // Number of RGB LEDs

const int RGB_UP_PIN = 4;    // RGB D4
const int RGB_UP_COUNT = 12; // Number of RGB LEDs

const int BUTTON_PIN = 3; // BUTTON_PIN D3

// LoRa configuration
#define LORA_CS 10
#define LORA_DIO0 12
#define LORA_FREQUENCY 433.0 // 433 MHz
#define LORA_SYNC_WORD 0x12
#define PACKET_LEN 46 // Fixed packet size
#define LORA_TRANSMIT_INTERVAL 250
// Sensor addresses
#define BME280_ADD 0x76
#define BMP280_ADD 0x77
#define MPU6050_ADDRESS 0x68

// Thresholds
const float LAUNCH_ACCEL_THRESHOLD = 1.5;  // g
const float LANDING_ACCEL_THRESHOLD = 0.2; // g
const int APOGEE_DELAY_MS = 2000;

#define RED 0xFF2D00     // Bright red
#define GREEN 0x00D26A   // Vibrant green
#define BLUE 0x007BFF    // Rich blue
#define YELLOW 0xFFD700  // Warm golden yellow
#define CYAN 0x00FFFF    // Electric cyan
#define MAGENTA 0xE100FF // Vivid magenta
#define WHITE 0xFFFFFF   // Pure white
#define ORANGE 0xFF6F00  // Deep orange
#define PURPLE 0x7D00A3  // Royal purple
#define PINK 0xFF69B4    // Soft pink
#define TEAL 0x008080    // Classic teal
