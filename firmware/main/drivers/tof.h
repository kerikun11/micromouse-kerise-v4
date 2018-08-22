#pragma once

#include "VL6180X.h"
#include "i2c.h"

#define TOF_TASK_PRIORITY 1
#define TOF_TASK_STACK_SIZE 4096

class ToF {
public:
  ToF(i2c_port_t i2c_port) : sensor(i2c_port) {}
  bool begin() {
    sensor.setTimeout(10);
    sensor.init();
    sensor.configureDefault();
    xTaskCreate([](void *obj) { static_cast<ToF *>(obj)->task(); }, "ToF",
                TOF_TASK_STACK_SIZE, this, TOF_TASK_PRIORITY, NULL);
    if (sensor.last_status != 0) {
      log_e("ToF failed :(");
      return false;
    }
    return true;
  }
  uint16_t getDistance() { return distance; }
  uint16_t passedTimeMs() { return passed_ms; }
  void print() { log_d("ToF: %d [mm]", getDistance()); }
  void csv() {
    printf("0,45,90,135,180,%d,%d\n", getDistance(), passed_ms);
    //      printf("0,90,180,270,360,%d\n", getDistance());
    //      printf("0,20,40,%d\n", passed_ms);
  }

private:
  VL6180X sensor;
  uint16_t distance;
  uint16_t passed_ms;

  void task() {
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
      sensor.writeReg(VL6180X::SYSRANGE__START, 0x01);
      {
        uint32_t startAt = millis();
        while ((sensor.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO) &
                0x04) == 0) {
          xLastWakeTime = xTaskGetTickCount();
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
          passed_ms++;
          if (millis() - startAt > 100)
            break;
        }
      }
      uint16_t range = sensor.readReg(VL6180X::RESULT__RANGE_VAL);
      sensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);
      distance = range + 20;
      if (range != 255) {
        passed_ms = 0;
      }
    }
  }
};
