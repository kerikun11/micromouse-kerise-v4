/**
 * @file fan.h
 * @brief Fan Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include "peripheral/mcpwm_kerise.h"

namespace hardware {

class Fan {
 public:
  Fan(gpio_num_t gpio_num,
      mcpwm_unit_t unit = MCPWM_UNIT_1,
      mcpwm_timer_t timer = MCPWM_TIMER_0,
      mcpwm_io_signals_t io_signals = MCPWM0A)
      : gpio_num(gpio_num), unit(unit), timer(timer) {
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, io_signals, gpio_num));
    mcpwm_config_t pwm_config = {
        .frequency = 250000,  //< frequency (period: 640 <= 160MHz/250kHz)
        .cmpr_a = 0,          //< duty cycle of PWMxA = 0
        .cmpr_b = 0,          //< duty cycle of PWMxB = 0
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    ESP_ERROR_CHECK(mcpwm_init(unit, timer, &pwm_config));
  }
  ~Fan() {
    mcpwm_stop(unit, timer);
    gpio_reset_pin(gpio_num);
  }
  void drive(float duty) {
    float duty_cycle = duty * 100;  //< from [0,1] to [0,100]
    duty_cycle = std::min(duty_cycle, 100.0f);
    duty_cycle = std::max(duty_cycle, 0.0f);
    mcpwm_set_duty(unit, timer, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(unit, timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  }
  void free() { mcpwm_set_signal_low(unit, timer, MCPWM_OPR_A); }

 private:
  gpio_num_t gpio_num;
  mcpwm_unit_t unit;
  mcpwm_timer_t timer;
};

};  // namespace hardware
