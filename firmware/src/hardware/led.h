#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <peripheral/i2c.h>

class LED {
private:
  static constexpr uint8_t PCA9632_DEV_ID = 0x62; //全体制御用のI2Cアドレス
  static constexpr int LED_STACK_SIZE = 2048;
  static constexpr int LED_TASK_PRIORITY = 1;

public:
  LED(i2c_port_t i2c_port) : i2c_port(i2c_port) {
    playList = xQueueCreate(5, sizeof(uint8_t));
  }
  bool init() {
    writeReg(0x00, 0b10000001);
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "LED", LED_STACK_SIZE, this, LED_TASK_PRIORITY, NULL);
    return true;
  }
  operator uint8_t() const { return value; }
  uint8_t operator=(uint8_t new_value) {
    value = new_value;
    xQueueSendToBack(playList, &value, 0);
    return value;
  }

private:
  const i2c_port_t i2c_port;
  QueueHandle_t playList;
  uint8_t value;

  void task() {
    while (1) {
      uint8_t value;
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
    return I2C::writeReg8(i2c_port, PCA9632_DEV_ID, reg, &data, 1);
  }
};
