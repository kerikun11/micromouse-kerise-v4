/**
 * @file VL6180X.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief C++ Library for VL6180X with ESP-IDF
 * @version 0.1
 * @date 2018-11-07
 *
 * @copyright Copyright (c) 2018 Ryotaro Onuki
 *
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2c.h"

#include "vl6180x_api.h"
#include "vl6180x_def.h"
#include "vl6180x_platform.h"

#include "esp_log.h"
#define TAG "VL6180X"

static constexpr uint8_t VL6180x_I2C_ADDRESS_DEFAULT = 0x29;

/**
 * @brief VL6180X class
 *
 */
class VL6180X {
public:
  VL6180X(i2c_port_t i2c_port = I2C_NUM_0) : i2c_port(i2c_port) {}

  bool init() {
    /* device init */
    dev = &myDev;
    dev->i2c_port_num = i2c_port;
    dev->i2c_address = VL6180x_I2C_ADDRESS_DEFAULT;
    if (VL6180x_InitData(dev) < 0) {
      ESP_LOGE(TAG, "InitData");
      return false;
    }
    if (VL6180x_Prepare(dev) < 0) {
      ESP_LOGE(TAG, "Prepare");
      return false;
    }
    return true;
  }
  int read() {
    int32_t RangeMilliMeter = INT32_MAX;
    if (!read(&RangeMilliMeter))
      return INT_MAX;
    return RangeMilliMeter;
  }
  bool read(int32_t *pRangeMilliMeter) {
    VL6180x_RangeData_t Range;
    if (VL6180x_RangePollMeasurement(dev, &Range) < 0)
      return false;
    if (Range.errorStatus != 0)
      return false;
    *pRangeMilliMeter = Range.range_mm;
    return true;
  }
  void i2cMasterInit(gpio_num_t pin_sda = GPIO_NUM_21,
                     gpio_num_t pin_scl = GPIO_NUM_22, uint32_t freq = 400000) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = pin_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = pin_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;
    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));
  }

protected:
  i2c_port_t i2c_port;
  VL6180xDev_t dev;
  MyDev_t myDev;
  int32_t TimingBudgetMicroSeconds;
};

#undef TAG
