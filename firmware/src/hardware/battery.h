#pragma once

#include <Arduino.h>

class Battery {
public:
  Battery(int8_t pin) : pin(pin) { adcAttachPin(pin); }
  float get_voltage() const { return v; }
  void update() {
    adcAttachPin(pin);
    v = 2 * 1.1f * 3.54813389f * adcEnd(pin) / 4095;
  }

private:
  int8_t pin;
  float v = 0;
};
