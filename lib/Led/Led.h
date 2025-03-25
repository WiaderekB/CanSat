#include <vector>
void setupLed();
void startupBlink();
void blinkDone();

typedef enum
{
  DOWN,
  UP
} Leds;

void setLed(uint32_t color, std::vector<uint8_t> id, Leds leds = Leds::UP);
void clearLed(Leds leds = Leds::UP);
