#include <Arduino.h>
#include "../../include/header.h"

void setupBuzzer()
{
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is OFF at startup
}

void startupSound()
{
}