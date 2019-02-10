#pragma once

#include "driver/mcpwm.h"

class OneMotor {
public:
  OneMotor(mcpwm_unit_t unit, mcpwm_timer_t timer,
           mcpwm_io_signals_t io_signals_1, mcpwm_io_signals_t io_signals_2,
           gpio_num_t gpio_num_1, gpio_num_t gpio_num_2)
      : unit(unit), timer(timer) {
    mcpwm_gpio_init(unit, io_signals_1, gpio_num_1);
    gpio_pulldown_en(gpio_num_1);
    mcpwm_gpio_init(unit, io_signals_2, gpio_num_2);
    gpio_pulldown_en(gpio_num_2);
    mcpwm_config_t pwm_config = {0};
    pwm_config.frequency = 100000; //< frequency
    pwm_config.cmpr_a = 0;         //< duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;         //< duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    ESP_ERROR_CHECK(mcpwm_init(unit, timer, &pwm_config));
  }
  void free() {
    mcpwm_set_signal_low(unit, timer, MCPWM_OPR_A);
    mcpwm_set_signal_low(unit, timer, MCPWM_OPR_B);
  }
  void drive(float duty) {
    float duty_cycle = duty * 100; //< [-1,1] to [-100,100]
    duty_cycle = std::min(duty_cycle, 100.0f);
    duty_cycle = std::max(duty_cycle, -100.0f);
    if (duty > 0) {
      mcpwm_set_signal_high(unit, timer, MCPWM_OPR_B);
      mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 100.0f - duty_cycle);
      mcpwm_set_duty_type(unit, timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    } else {
      mcpwm_set_signal_high(unit, timer, MCPWM_OPR_A);
      mcpwm_set_duty(unit, timer, MCPWM_OPR_B, 100.0f + duty_cycle);
      mcpwm_set_duty_type(unit, timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
  }

private:
  mcpwm_unit_t unit;
  mcpwm_timer_t timer;
};

class Motor {
private:
  static constexpr float emergency_threshold = 2.0f;

public:
  Motor(gpio_num_t gpio_L1, gpio_num_t gpio_L2, gpio_num_t gpio_R1,
        gpio_num_t gpio_R2)
      : mt_L(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, gpio_L1, gpio_L2),
        mt_R(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B, gpio_R1, gpio_R2) {
    emergency = false;
    free();
  }
  void drive(float valueL, float valueR) {
    if (emergency)
      return;
    if (std::abs(valueL) > emergency_threshold ||
        std::abs(valueR) > emergency_threshold)
      emergencyStop();
    mt_L.drive(valueL);
    mt_R.drive(valueR);
  }
  void free() {
    mt_L.free();
    mt_R.free();
  }
  void emergencyStop() {
    emergency = true;
    free();
  }
  void emergencyRelease() {
    emergency = false;
    free();
  }
  bool isEmergency() { return emergency; }

private:
  bool emergency;
  OneMotor mt_L;
  OneMotor mt_R;
};
