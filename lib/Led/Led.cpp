#include <Arduino.h>
#include "../../include/header.h"
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels(RGB_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);

void setupLed()
{
  pixels.begin();
}

void startupBlink()
{
  pixels.clear();

  for (int i = 0; i < RGB_COUNT / 2; i++)
  {
    pixels.setPixelColor(i, pixels.Color(150, 0, 0));
    pixels.setPixelColor(i + 6, pixels.Color(150, 0, 0));
    pixels.show();
    delay(75);
  }

  delay(150);
  for (int i = 0; i < RGB_COUNT; i++)
  {
    pixels.setPixelColor(i, pixels.Color(150, 150, 150));
  }
  pixels.show();
}

void blinkDone()
{
  for (int i = 11; i >= RGB_COUNT / 2; i--)
  {
    pixels.setPixelColor(i, pixels.Color(150, 0, 0));
    pixels.setPixelColor(i - 6, pixels.Color(150, 0, 0));
    pixels.show();
    delay(75);
  }
  delay(50);

  pixels.clear();
  pixels.show();
}
