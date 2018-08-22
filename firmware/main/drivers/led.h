#pragma once

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "i2c.h"

#define PCA9632_DEV_ID 0x62 //全体制御用のI2Cアドレス
#define LED_STACK_SIZE 2048
#define LED_TASK_PRIORITY 1

class LED {
public:
  LED(i2c_port_t i2c_port) : i2c_port(i2c_port) {
    playList = xQueueCreate(5, sizeof(uint8_t));
  }
  bool begin() {
    writeReg(0x00, 0b10000001);
    xTaskCreate([](void *obj) { static_cast<LED *>(obj)->task(); }, "LED",
                LED_STACK_SIZE, this, LED_TASK_PRIORITY, NULL);
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
  xQueueHandle playList;
  uint8_t value;

  void task() {
    while (1) {
      uint8_t value;
      if (xQueueReceive(playList, &value, portMAX_DELAY) == pdTRUE) {
        writeValue(value);
      }
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
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (PCA9632_DEV_ID << 1) | I2C_MASTER_WRITE,
    // true); i2c_master_write_byte(cmd, reg, true); i2c_master_write_byte(cmd,
    // data, true); i2c_master_stop(cmd); esp_err_t ret =
    // i2c_master_cmd_begin(i2c_port, cmd, 1 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // return ret == ESP_OK;
    return I2C::writeReg8(i2c_port, PCA9632_DEV_ID, reg, &data, 1);
  }
};
