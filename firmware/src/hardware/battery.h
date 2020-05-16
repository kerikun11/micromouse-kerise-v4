#pragma once

#include <Arduino.h>

class Battery {
public:
  Battery(int8_t pin) : pin(pin) {}
  float get_voltage() {
    return 2 * 1.1f * 3.54813389f * analogRead(pin) / 4095;
  }

private:
  int8_t pin;
};
