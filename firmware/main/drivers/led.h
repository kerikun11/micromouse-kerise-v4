#pragma once

#include <driver/i2c.h>

#define PCA9632_DEV_ID 0x62 //全体制御用のI2Cアドレス

class LED {
  public:
    LED(const i2c_port_t i2c_port) : i2c_port(i2c_port) {}
    bool begin(const int8_t pin_sda, const int8_t pin_scl) {
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)pin_sda;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_io_num = (gpio_num_t)pin_scl;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 1000000;
        i2c_param_config(i2c_port, &conf);
        i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
        return begin();
    }
    bool begin() {
      writeReg(0x00, B10000001);
      return true;
    }
    operator uint8_t() const {
      return value;
    }
    uint8_t operator=(uint8_t new_value) {
      value = new_value;
      uint8_t data = 0;
      data |= (value & 1) << 0;
      data |= (value & 2) << 1;
      data |= (value & 4) << 2;
      data |= (value & 8) << 3;
      writeReg(0x08, data);
      return value;
    }
  private:
    const i2c_port_t i2c_port;
    uint8_t value;
    bool writeReg(uint8_t reg, uint8_t data) {
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (PCA9632_DEV_ID << 1) | I2C_MASTER_WRITE, true);
      i2c_master_write_byte(cmd, reg, true);
      i2c_master_write_byte(cmd, data, true);
      i2c_master_stop(cmd);
      esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd);
      return ret == ESP_OK;
    }
};

