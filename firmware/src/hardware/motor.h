/**
 * @file motor.h
 * @brief Motor Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <peripheral/mcpwm_kerise.h>
#include <algorithm>  //< std::max(), std::min()
#include <cmath>      //< std::isfinit()
#include <iostream>

namespace hardware {

class OneMotor {
 public:
  OneMotor(mcpwm_unit_t unit,
           mcpwm_timer_t timer,
           mcpwm_io_signals_t io_signals_1,
           mcpwm_io_signals_t io_signals_2,
           gpio_num_t gpio_num_1,
           gpio_num_t gpio_num_2)
      : unit(unit), timer(timer) {
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, io_signals_1, gpio_num_1));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, io_signals_2, gpio_num_2));
    mcpwm_config_t pwm_config = {
        .frequency = 250000,  //< frequency (period: 640 @ 160MHz/250kHz)
        .cmpr_a = 0,          //< duty cycle of PWMxA = 0
        .cmpr_b = 0,          //< duty cycle of PWMxB = 0
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    ESP_ERROR_CHECK(mcpwm_init(unit, timer, &pwm_config));
  }
  void free() {
    mcpwm_set_signal_low(unit, timer, MCPWM_OPR_A);
    mcpwm_set_signal_low(unit, timer, MCPWM_OPR_B);
  }
  void drive(float duty) {
    if (!std::isfinite(duty)) {
      std::cout << __FILE__ ":" << __LINE__ << "\t" << duty << std::endl;
      free();
      return;
    }
    // const float duty_cycle = std::clamp(duty * 100, -100, 100);
    float duty_cycle = duty * 100;  //< [-1,1] to [-100,100]
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
  static constexpr float emergency_threshold = 1.3f;

 public:
  Motor(gpio_num_t gpio_L1,
        gpio_num_t gpio_L2,
        gpio_num_t gpio_R1,
        gpio_num_t gpio_R2)
      : mt_L(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, gpio_L1, gpio_L2),
        mt_R(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B, gpio_R1, gpio_R2) {
    free();
  }
  void drive(float valueL, float valueR) {
    if (emergency)
      return;
    mt_L.drive(valueL);
    mt_R.drive(valueR);
    if (std::abs(valueL) > emergency_threshold ||
        std::abs(valueR) > emergency_threshold)
      emergency_stop();
  }
  void free() {
    mt_L.free();
    mt_R.free();
  }
  void emergency_stop() {
    emergency = true;
    free();
  }
  void emergency_release() {
    emergency = false;
    free();
  }
  bool is_emergency() const { return emergency; }

 private:
  bool emergency = false;
  OneMotor mt_L;
  OneMotor mt_R;
};

};  // namespace hardware
