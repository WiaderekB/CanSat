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
    ledDown.setPixelColor(i, YELLOW);
    ledDown.setPixelColor(i + 6, YELLOW);
    if (i % 3 == 0)
    {
      ledUp.setPixelColor(int(i / 3), YELLOW);
      ledUp.show();
    }
    ledDown.show();
    delay(75);
  }
}

void blinkDone(bool success)
{
  auto color = success ? GREEN : RED;
  for (int i = 0; i < RGB_DOWN_COUNT / 2; i++)
  {
    ledDown.setPixelColor(i, color);
    ledDown.setPixelColor(i + 6, color);
    if (i % 3 == 0)
    {
      ledUp.setPixelColor(int(i / 3), color);
    }
  }

  ledUp.show();
  ledDown.show();
  delay(50);
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