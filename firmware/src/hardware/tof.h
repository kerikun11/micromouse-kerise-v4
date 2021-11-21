/**
 * @file tof.h
 * @brief ToF Sensor Driver
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include <VL6180X.h>
#include <cstdio>
#include <ctrl/accumulator.h>
#include <peripheral/i2c.h>

class ToF {
public:
  static constexpr UBaseType_t Priority = 1;

public:
  ToF(i2c_port_t i2c_port, float tof_dist_offset)
      : sensor(i2c_port), tof_dist_offset(tof_dist_offset) {}
  bool init() {
    sensor.setTimeout(20);
    sensor.init();
    /* [pre-cal] fixed 3.2ms */
    sensor.configureDefault();
    /* [readout average; 1300us + 64.5us * value] default: 48 (4.3ms) */
    // sensor.writeReg(VL6180X::READOUT__AVERAGING_SAMPLE_PERIOD, 32); //< 3.2ms
    // sensor.writeReg(VL6180X::READOUT__AVERAGING_SAMPLE_PERIOD, 64); //< 5.4ms
    /* [max-convergence; includes readout average time] default: 49ms */
    sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 32);
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "ToF", 4096, this, Priority, NULL);
    vTaskDelay(pdMS_TO_TICKS(40));
    if (sensor.last_status != 0) {
      log_e("ToF failed :(");
      return false;
    }
    return true;
  }
  void enable() { enabled = true; }
  void disable() { enabled = false; }
  uint16_t getDistance() const { return distance; }
  const auto &getLog() const { return log; }
  uint16_t passedTimeMs() const { return passed_ms; }
  bool isValid() const { return passed_ms < 20; }
  void print() const {
    log_i("ToF: %3d [mm] Dur: %3d [ms], Passed: %4d [ms]", getDistance(), dur,
          passedTimeMs());
  }
  void csv() const {
    std::printf("0,45,90,135,180,%d,%d\n", getDistance(), passedTimeMs());
  }

private:
  VL6180X sensor;
  float tof_dist_offset;
  volatile bool enabled = true;
  uint16_t distance;
  uint16_t dur;
  int passed_ms;
  ctrl::Accumulator<uint16_t, 10> log;

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      if (!enabled) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
        passed_ms++;
        continue;
      }
      /* sampling start */
      sensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);
      sensor.writeReg(VL6180X::SYSRANGE__START, 0x01);
      {
        uint32_t startAt = millis();
        while ((sensor.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO) &
                0x04) == 0) {
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
          passed_ms++;
          if (millis() - startAt > 100)
            break;
        }
        dur = millis() - startAt;
      }
      // uint8_t range_status = sensor.readReg(VL6180X::RESULT__RANGE_STATUS) >>
      // 4;
      /* get data from sensor */
      uint16_t range = sensor.readReg(VL6180X::RESULT__RANGE_VAL);
      distance = (range + tof_dist_offset - 90) * model::tof_dist_factor + 90;
      log.push(distance);
      if (range != 255)
        passed_ms = 0;
    }
  }
};
