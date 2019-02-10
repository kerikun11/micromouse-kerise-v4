#pragma once

#include "app_log.h"
#include "i2c.h"

#include "vl6180x_api.h"
#include "vl6180x_def.h"
#include "vl6180x_platform.h"

#define TOF_TASK_PRIORITY 1
#define TOF_TASK_STACK_SIZE 4096

#define TOF_DISTANCE_OFFSET (26)

class ToF {
  static constexpr uint8_t VL6180x_I2C_ADDRESS_DEFAULT = 0x29;

public:
  ToF(i2c_port_t i2c_port) : i2c_port(i2c_port) {}
  bool begin() {
    /* device init */
    dev = &myDev;
    dev->i2c_port_num = i2c_port;
    dev->i2c_address = VL6180x_I2C_ADDRESS_DEFAULT;
    if (VL6180x_InitData(dev) < 0) {
      loge << "Failed to initialize ToF :(" << std::endl;
      return false;
    }
    if (VL6180x_Prepare(dev) < 0) {
      loge << "Failed to prepare ToF :(" << std::endl;
      return false;
    }
    xTaskCreate([](void *obj) { static_cast<ToF *>(obj)->task(); }, "ToF",
                TOF_TASK_STACK_SIZE, this, TOF_TASK_PRIORITY, NULL);
    enabled = true;
    return true;
  }
  void enable() { enabled = true; }
  void disable() { enabled = false; }
  uint16_t getDistance() { return distance; }
  uint16_t passedTimeMs() { return passed_ms; }
  void print() {
    log_d("ToF: %d [mm], Passed %d [ms]", getDistance(), passedTimeMs());
  }
  void csv() {
    printf("0,45,90,135,180,%d,%d\n", getDistance(), passedTimeMs());
  }

private:
  i2c_port_t i2c_port;
  VL6180xDev_t dev;
  MyDev_t myDev;
  bool enabled = true;
  uint16_t distance;
  uint16_t passed_ms;

  void task() {
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
      if (!enabled) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        passed_ms++;
        continue;
      }
      VL6180x_RangeStartSingleShot(dev);
      VL6180x_RangeData_t Range;
      while (1) {
        int status = VL6180x_RangeGetMeasurementIfReady(dev, &Range);
        if (status == 0) {
          // 測定完了
          break;
        } else if (status == NOT_READY) {
          // 測定中
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
          passed_ms++;
        } else if (status < 0) {
          loge << "ToF Error :(" << std::endl;
        }
      }
      // 測定完了
      if (Range.errorStatus == 0) {
        // 測定成功
        auto range = Range.range_mm;
        distance = range + TOF_DISTANCE_OFFSET;
        if (range != 255)
          passed_ms = 0;
      }
    }
  }
};
