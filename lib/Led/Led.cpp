#include <Arduino.h>
#include <vector>
#include "../../include/header.h"
#include <Adafruit_NeoPixel.h>
#include "Led.h"

Adafruit_NeoPixel ledDown(RGB_DOWN_COUNT, RGB_DOWN_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledUp(RGB_UP_COUNT, RGB_UP_PIN, NEO_GRB + NEO_KHZ800);

void setupLed()
{
  ledDown.begin();
  ledUp.begin();
}

void startupBlink()
{
  ledDown.clear();

  for (int i = 0; i < RGB_DOWN_COUNT / 2; i++)
  {
    ledDown.setPixelColor(i, ledDown.Color(150, 0, 0));
    ledDown.setPixelColor(i + 6, ledDown.Color(150, 0, 0));
    ledDown.show();
    delay(75);
  }

  delay(150);
  for (int i = 0; i < RGB_DOWN_COUNT; i++)
  {
    ledDown.setPixelColor(i, ledDown.Color(150, 150, 150));
  }
  ledDown.show();
}

void blinkDone()
{
  for (int i = 11; i >= RGB_DOWN_COUNT / 2; i--)
  {
    ledDown.setPixelColor(i, ledDown.Color(150, 0, 0));
    ledDown.setPixelColor(i - 6, ledDown.Color(150, 0, 0));
    ledDown.show();
    delay(75);
  }
  delay(50);

  ledDown.clear();
  ledDown.show();
}

void setLed(uint32_t color, std::vector<uint8_t> id, Leds leds)
{
  for (auto i : id)
  {
    if (leds == Leds::DOWN)
    {
      ledDown.setPixelColor(i, color);
    }
    else
    {
      ledUp.setPixelColor(i, color);
    }
  }
}

void clearLed(Leds leds)
{
  if (leds == Leds::DOWN)
  {
    ledDown.clear();
    ledDown.show();
  }
  else
  {
    ledUp.clear();
    ledUp.show();
  }
}