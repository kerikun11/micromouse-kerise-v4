#pragma once

#include "app_log.h"
#include "config/model.h" //< for KERISE_SELECT

#include <as5048a.h>
#include <freertospp/semphr.h>
#include <ma730.h>

#include <cmath>
#include <iomanip>

class Encoder {
public:
  Encoder(const float encoder_factor) : encoder_factor(encoder_factor) {}
#if KERISE_SELECT == 4 || KERISE_SELECT == 3
  bool init(spi_host_device_t spi_host, int8_t pin_cs) {
    if (!as.init(spi_host, pin_cs)) {
      app_loge << "AS5048A init failed :(" << std::endl;
      return false;
    }
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "Encoder", configMINIMAL_STACK_SIZE, this, Priority, NULL);
    return true;
  }
#elif KERISE_SELECT == 5
  bool init(spi_host_device_t spi_host, std::array<gpio_num_t, 2> pins_cs) {
    if (!ma[0].init(spi_host, pins_cs[0])) {
      app_loge << "Encoder L init failed :(" << std::endl;
      return false;
    }
    if (!ma[1].init(spi_host, pins_cs[1])) {
      app_loge << "Encoder R init failed :(" << std::endl;
      return false;
    }
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "Encoder", configMINIMAL_STACK_SIZE, this, Priority, NULL);
    return true;
  }
#endif
  int get_raw(uint8_t ch) const { return pulses_raw[ch]; }
  float get_position(uint8_t ch) const {
    /* the reason the type of pulses is no problem with type int */
    /* estimated position 1,000 [mm/s] * 10 [min] * 60 [s/min] = 600,000 [mm] */
    /* int32_t 2,147,483,647 / 16384 * 1/3 * 3.1415 * 13 [mm] = 1,784,305 [mm]*/
    return positions[ch];
  }
  void clearOffset() { pulses_ovf[0] = pulses_ovf[1] = 0; }
  void csv() {
    // std::cout << "0," << get_position(0) << "," << get_position(1) <<
    // std::endl;
    // std::cout << "0," << get_raw(0) << "," << get_raw(1) << std::endl;
    std::cout << -pulses[0] << "," << pulses[1] << std::endl;
#if 0
    static int pre[2];
    int now[2];
    for (int i : {0, 1})
      now[i] = pulses[i];
    std::cout << -(now[0] - pre[0]) << "," << now[1] - pre[1] << std::endl;
    for (int i : {0, 1})
      pre[i] = now[i];
#endif
  }
  friend std::ostream &operator<<(std::ostream &os, const Encoder &e) {
    return os << "Encoder: position (" //
              << std::setw(6) << std::setfill(' ') << e.get_position(0) << ", "
              << std::setw(6) << std::setfill(' ') << e.get_position(1) << ")"
              << " pulses_raw (" //
              << std::setw(6) << std::setfill(' ') << e.get_raw(0) << ", "
              << std::setw(6) << std::setfill(' ') << e.get_raw(1) << ")";
  }
  void sampling_sync(portTickType xBlockTime = portMAX_DELAY) const {
    sampling_end_semaphore.take(xBlockTime);
  }

private:
  static constexpr int Priority = 7;
#if KERISE_SELECT == 3 || KERISE_SELECT == 4
  AS5048A_DUAL as;
#elif KERISE_SELECT == 5
  MA730 ma[2];
#endif
  const float encoder_factor;
  int pulses[2];
  int pulses_raw[2];
  int pulses_prev[2];
  int pulses_ovf[2];
  float positions[2];
  freertospp::Semaphore sampling_end_semaphore;

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();
      sampling_end_semaphore.give();
    }
  }

  void update() {
#if KERISE_SELECT == 3 || KERISE_SELECT == 4
    constexpr int pulses_size = AS5048A_DUAL::PULSES_SIZE;
    as.update();
    for (int i = 0; i < 2; i++)
      pulses_raw[i] = as.get(i);
#elif KERISE_SELECT == 5
    constexpr int pulses_size = MA730::PULSES_SIZE;
    for (int i = 0; i < 2; i++) {
      ma[i].update();
      pulses_raw[i] = ma[i].get();
    }
    pulses_raw[0] +=
        9.5e-3f * pulses_size *
            std::sin(2 * M_PI *
                     (float(pulses_raw[0]) / pulses_size - 0.1e-1f + 0.5f)) -
        (9.7732187836986f);
    pulses_raw[1] +=
        8.0e-3f * pulses_size *
            std::sin(2 * M_PI *
                     (float(pulses_raw[1]) / pulses_size - 9.9e-1f + 0.5f)) -
        (-8.23007897574619f);
#endif
    float mm[2];
    for (int i = 0; i < 2; i++) {
      /* count overflow */
      if (pulses_raw[i] > pulses_prev[i] + pulses_size / 2) {
        pulses_ovf[i]--;
      } else if (pulses_raw[i] < pulses_prev[i] - pulses_size / 2) {
        pulses_ovf[i]++;
      }
      pulses_prev[i] = pulses_raw[i];
      /* calculate position */
      pulses[i] = pulses_ovf[i] * pulses_size + pulses_raw[i];
      mm[i] =
          (pulses_ovf[i] + float(pulses_raw[i]) / pulses_size) * encoder_factor;
    }
    /* fix rotation direction */
#if KERISE_SELECT == 3 || KERISE_SELECT == 4
    positions[0] = +mm[0];
    positions[1] = -mm[1];
#elif KERISE_SELECT == 5
    positions[0] = -mm[0];
    positions[1] = +mm[1];
#endif
  }
};
