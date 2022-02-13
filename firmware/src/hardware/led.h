/**
 * @file led.h
 * @brief LED Driver
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <peripheral/i2c.h>

namespace hardware {

class LED {
private:
  static constexpr uint8_t PCA9632_DEV_ID = 0x62; //< 全体制御用のI2Cアドレス

public:
  LED() { playList = xQueueCreate(/* uxQueueLength = */ 5, sizeof(uint8_t)); }
  bool init(i2c_port_t i2c_port) {
    this->i2c_port = i2c_port;
    writeReg(0x00, 0b10000001);
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "LED", 2048, this, 1, NULL);
    return true;
  }
  uint8_t set(uint8_t new_value) {
    value = new_value;
    xQueueSendToBack(playList, &value, 0);
    return value;
  }
  uint8_t get() const { return value; }
  operator uint8_t() const { return value; }
  uint8_t operator=(uint8_t new_value) { return set(new_value); }

private:
  i2c_port_t i2c_port;
  QueueHandle_t playList;
  uint8_t value;

  void task() {
    while (1) {
      if (xQueueReceive(playList, &value, portMAX_DELAY) == pdTRUE)
        writeValue(value);
    }
  }
  bool writeValue(uint8_t value) {
    uint8_t data = 0;
    data |= (value & 1) << 0;
    data |= (value & 2) << 1;
    data |= (value & 4) << 2;
    data |= (value & 8) << 3;
    return writeReg(0x08, data);
  }
  bool writeReg(uint8_t reg, uint8_t data) {
    return peripheral::I2C::writeReg8(i2c_port, PCA9632_DEV_ID, reg, &data, 1,
                                      pdMS_TO_TICKS(10));
  }
};

}; // namespace hardware
