#include <stdint.h>
#include "StateMachine.h"

class Button
{
public:
  Button(StateMachine &stateMachine);

  void init();
  void checkButton(const ProbeState &state);

private:
  bool pressed;
  uint32_t pressedSince;
  StateMachine &stateMachine;
};