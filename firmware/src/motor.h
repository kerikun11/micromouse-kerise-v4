#pragma once

#include <Arduino.h>

class Motor {
public:
  static constexpr int MOTOR_CTRL_FREQUENCY = 250000;
  static constexpr int MOTOR_CTRL_BIT_NUM = 10;
  static constexpr int MOTOR_DUTY_MAX = 1023; //< 2 ^ MOTOR_CTRL_BIT_NUM - 1
  static constexpr int MOTOR_DUTY_SAT = 1023;

public:
  Motor(uint8_t pin_L1, uint8_t pin_L2, uint8_t pin_R1, uint8_t pin_R2,
        uint8_t ch_L1, uint8_t ch_L2, uint8_t ch_R1, uint8_t ch_R2)
      : ch_L1(ch_L1), ch_L2(ch_L2), ch_R1(ch_R1), ch_R2(ch_R2) {
    emergency = false;
    ledcSetup(ch_L1, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
    ledcSetup(ch_L2, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
    ledcSetup(ch_R1, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
    ledcSetup(ch_R2, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
    ledcAttachPin(pin_L1, ch_L1);
    ledcAttachPin(pin_L2, ch_L2);
    ledcAttachPin(pin_R1, ch_R1);
    ledcAttachPin(pin_R2, ch_R2);
    free();
  }
  void left(int duty) {
    if (emergency)
      return;
    if (duty > MOTOR_DUTY_SAT) {
      ledcWrite(ch_L1, 0);
      ledcWrite(ch_L2, MOTOR_DUTY_SAT);
    } else if (duty < -MOTOR_DUTY_SAT) {
      ledcWrite(ch_L1, MOTOR_DUTY_SAT);
      ledcWrite(ch_L2, 0);
    } else if (duty > 0) {
      ledcWrite(ch_L1, MOTOR_DUTY_MAX - duty);
      ledcWrite(ch_L2, MOTOR_DUTY_MAX);
    } else {
      ledcWrite(ch_L1, MOTOR_DUTY_MAX);
      ledcWrite(ch_L2, MOTOR_DUTY_MAX + duty);
    }
  }
  void right(int duty) {
    if (emergency)
      return;
    if (duty > MOTOR_DUTY_SAT) {
      ledcWrite(ch_R1, 0);
      ledcWrite(ch_R2, MOTOR_DUTY_SAT);
    } else if (duty < -MOTOR_DUTY_SAT) {
      ledcWrite(ch_R1, MOTOR_DUTY_SAT);
      ledcWrite(ch_R2, 0);
    } else if (duty > 0) {
      ledcWrite(ch_R1, MOTOR_DUTY_MAX - duty);
      ledcWrite(ch_R2, MOTOR_DUTY_MAX);
    } else {
      ledcWrite(ch_R1, MOTOR_DUTY_MAX);
      ledcWrite(ch_R2, MOTOR_DUTY_MAX + duty);
    }
  }
  void drive(int16_t valueL, int16_t valueR) {
    left(valueL);
    right(valueR);
  }
  void free() {
    ledcWrite(ch_L1, 0);
    ledcWrite(ch_L2, 0);
    ledcWrite(ch_R1, 0);
    ledcWrite(ch_R2, 0);
  }
  void emergency_stop() {
    emergency = true;
    free();
  }
  void emergency_release() {
    emergency = false;
    free();
  }
  bool isEmergency() { return emergency; }

private:
  uint8_t ch_L1, ch_L2, ch_R1, ch_R2;
  bool emergency;
};
