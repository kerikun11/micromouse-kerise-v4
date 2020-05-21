#pragma once

#include "app_log.h"

#include <as5048a.h>
#include <ma730.h>

#include <cmath>
#include <iomanip>

#include "config/model.h" //< for KERISE_SELECT

class Encoder {
public:
  Encoder(const float encoder_factor) : encoder_factor(encoder_factor) {}
#if KERISE_SELECT == 4 || KERISE_SELECT == 3
  bool init(spi_host_device_t spi_host, int8_t pin_cs) {
    if (!as.init(spi_host, pin_cs)) {
      loge << "AS5048A init failed :(" << std::endl;
      return false;
    }
    return true;
  }
#elif KERISE_SELECT == 5
  bool init(spi_host_device_t spi_host, std::array<gpio_num_t, 2> pins_cs) {
    if (!ma[0].init(spi_host, pins_cs[0])) {
      loge << "Encoder L init failed :(" << std::endl;
      return false;
    }
    if (!ma[1].init(spi_host, pins_cs[1])) {
      loge << "Encoder R init failed :(" << std::endl;
      return false;
    }
    return true;
  }
#endif
  int get_raw(uint8_t ch) const { return pulses[ch]; }
  float get_position(uint8_t ch) const {
    /* the reason the type of pulses is no problem with type int */
    /* estimated position 1,000 [mm/s] * 10 [min] * 60 [s/min] = 600,000 [mm] */
    /* int32_t 2,147,483,647 / 16384 * 1/3 * 3.1415 * 13 [mm] = 1,784,305 [mm]*/
    return positions[ch];
  }
  void clearOffset() { pulses_ovf[0] = pulses_ovf[1] = 0; }
  void csv() {
    std::cout << "0," << get_position(0) << "," << get_position(1) << std::endl;
    // std::cout << "0," << get_raw(0) << "," << get_raw(1) << std::endl;
  }
  friend std::ostream &operator<<(std::ostream &os, const Encoder &e) {
    return os << "Encoder: position (" //
              << std::setw(6) << std::setfill(' ') << e.get_position(0) << ", "
              << std::setw(6) << std::setfill(' ') << e.get_position(1) << ")"
              << " pulses (" //
              << std::setw(6) << std::setfill(' ') << e.get_raw(0) << ", "
              << std::setw(6) << std::setfill(' ') << e.get_raw(1) << ")";
  }

  void update() {
#if KERISE_SELECT == 3 || KERISE_SELECT == 4
    int pulses_size = AS5048A_DUAL::PULSES_SIZE;
    as.update();
    for (int i = 0; i < 2; i++)
      pulses[i] = as.get(i);
#elif KERISE_SELECT == 5
    int pulses_size = MA730::PULSES_SIZE;
    for (int i = 0; i < 2; i++) {
      ma[i].update();
      pulses[i] = ma[i].get();
    }
#endif
    for (int i = 0; i < 2; i++) {
      /* count overflow */
      if (pulses[i] > pulses_prev[i] + pulses_size / 2) {
        pulses_ovf[i]--;
      } else if (pulses[i] < pulses_prev[i] - pulses_size / 2) {
        pulses_ovf[i]++;
      }
      pulses_prev[i] = pulses[i];
      /* calculate position */
      positions[i] =
          (pulses_ovf[i] + float(pulses[i]) / pulses_size) * encoder_factor;
    }
    /* fix rotation direction */
#if KERISE_SELECT == 3 || KERISE_SELECT == 4
    positions[1] *= -1;
#elif KERISE_SELECT == 5
    positions[0] *= -1;
#endif
  }

private:
#if KERISE_SELECT == 3 || KERISE_SELECT == 4
  AS5048A_DUAL as;
#elif KERISE_SELECT == 5
  MA730 ma[2];
#endif
  const float encoder_factor;
  int pulses[2];
  int pulses_prev[2];
  int pulses_ovf[2];
  float positions[2];
};
