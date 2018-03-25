#pragma once

#include <Arduino.h>
//#include <Wire.h>
#include "VL6180X.h"

#define TOF_TASK_PRIORITY     1
#define TOF_TASK_STACK_SIZE   4096
#define I2C_PORT_NUM_TOF    I2C_NUM_0

class ToF {
  public:
    ToF(const int pin_sda, const int pin_scl): pin_sda(pin_sda), pin_scl(pin_scl) {}
    bool begin(bool initializeWire = false) {
      //      if (initializeWire) Wire.begin(pin_sda, pin_scl);
      if (initializeWire) {
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)pin_sda;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_io_num = (gpio_num_t)pin_scl;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 1000000;
        i2c_param_config(I2C_PORT_NUM_TOF, &conf);
        i2c_driver_install(I2C_PORT_NUM_TOF, conf.mode, 0, 0, 0);
      }
      sensor.setTimeout(10);
      sensor.init();
      sensor.configureDefault();
      xTaskCreate([](void* obj) {
        static_cast<ToF*>(obj)->task();
      }, "ToF", TOF_TASK_STACK_SIZE, this, TOF_TASK_PRIORITY, NULL);
      if (sensor.last_status != 0) {
        log_e("ToF failed :(");
        return false;
      }
      return true;
    }
    uint16_t getDistance() {
      return distance;
    }
    uint16_t passedTimeMs() {
      return passed_ms;
    }
    void print() {
      log_d("ToF: %d [mm]", getDistance());
    }
    void csv() {
      printf("0,45,90,135,180,%d,%d\n", getDistance(), passed_ms);
      //      printf("0,90,180,270,360,%d\n", getDistance());
      //      printf("0,20,40,%d\n", passed_ms);
    }
  private:
    const int pin_sda, pin_scl;
    VL6180X sensor;
    uint16_t distance;
    uint16_t passed_ms;

    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        sensor.writeReg(VL6180X::SYSRANGE__START, 0x01);
        {
          uint32_t startAt = millis();
          while ((sensor.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO) & 0x04) == 0) {
            vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
            passed_ms++;
            if (millis() - startAt > 100) break;
          }
        }
        uint16_t range = sensor.readReg(VL6180X::RESULT__RANGE_VAL);
        sensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);
        distance = range + 24;
        if (range != 255) {
          passed_ms = 0;
        }
      }
    }
};

