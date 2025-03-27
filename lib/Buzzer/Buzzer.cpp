#include <Arduino.h>
#include "../../include/header.h"

void setupBuzzer()
{
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is OFF at startup
}

void startupSound(bool success)
{
  if (success)
  {
    // Short beep
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);

    // Two quick beeps
    digitalWrite(BUZZER_PIN, HIGH);
    delay(50);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(50);
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);

    // Long beep
    digitalWrite(BUZZER_PIN, HIGH);
    delay(300);
    digitalWrite(BUZZER_PIN, LOW);
  }
  else
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(25);
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);

    digitalWrite(BUZZER_PIN, HIGH);
    delay(450);
    digitalWrite(BUZZER_PIN, LOW);
    delay(5);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);

    while (true)
    {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(250);
      digitalWrite(BUZZER_PIN, LOW);
      delay(750);
    }
  }
}

void buzzerOn()
{
  digitalWrite(BUZZER_PIN, HIGH);
}

void buzzerOff()
{
  digitalWrite(BUZZER_PIN, LOW);
}