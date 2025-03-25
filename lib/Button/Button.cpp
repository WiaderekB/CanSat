#include "Button.h"
#include <Arduino.h>
#include "../../include/header.h"
#include "StateMachine.h"

Button::Button(StateMachine &stateMachine) : stateMachine(stateMachine) {}

void Button::init()
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}
void Button::checkButton(const ProbeState &state)
{
  if (digitalRead(BUTTON_PIN) == LOW)
  {

    if ((millis() - pressedSince) > BUTTON_THRESHOLD_MS && state == ProbeState::GROUND)
    {
      switch (state)
      {
      case ProbeState::GROUND:
        stateMachine.setProbeState(ProbeState::SOS);
        break;
      case ProbeState::STANDBY:
        stateMachine.setProbeState(ProbeState::ARMED);
        break;
      }
    }

    if (!pressed)
      pressedSince = millis();

    pressed = true;
  }
  else
  {
    pressed = false;
  }
}