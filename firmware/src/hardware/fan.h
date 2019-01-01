#pragma once

#include <Arduino.h>

class Fan {
public:
  static constexpr int FAN_FREQUENCY = 10000;
  static constexpr int FAN_BIT_NUM = 8;

public:
  Fan(int pin, uint8_t channel) : pin(pin), channel(channel) {
    ledcSetup(channel, FAN_FREQUENCY, FAN_BIT_NUM);
    ledcAttachPin(pin, channel);
    ledcWrite(channel, 0);
  }
  void drive(const float duty) {
    ledcWrite(channel, duty * (pow(2, FAN_BIT_NUM) - 1));
  }

private:
  int pin;
  uint8_t channel;
};
