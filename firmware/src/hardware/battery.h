#pragma once

#include <Arduino.h>

class Battery {
public:
  Battery(int8_t pin) : pin(pin) {}
  float get_voltage() {
    return 2 * float(1.1) * float(3.54813389) * analogRead(pin) / 4095;
  }

private:
  int8_t pin;
};
