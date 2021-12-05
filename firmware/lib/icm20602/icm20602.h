/**
 * @file icm20602.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief ICM-20602 Driver
 * @date 2020-05-23
 * @copyright Copyright (c) 2020 Ryotaro Onuki
 */
#pragma once

#include <array>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <peripheral/spi.h>

struct MotionParameter {
  float x, y, z;
  MotionParameter(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
  MotionParameter operator+(const MotionParameter &obj) const {
    return MotionParameter(x + obj.x, y + obj.y, z + obj.z);
  }
  MotionParameter operator-(const MotionParameter &obj) const {
    return MotionParameter(x - obj.x, y - obj.y, z - obj.z);
  }
  MotionParameter operator*(const float mul) const {
    return MotionParameter(x * mul, y * mul, z * mul);
  }
  MotionParameter operator/(const float div) const {
    return MotionParameter(x / div, y / div, z / div);
  }
  MotionParameter &operator+=(const MotionParameter &obj) {
    return x += obj.x, y += obj.y, z += obj.z, *this;
  }
  MotionParameter &operator-=(const MotionParameter &obj) {
    return x -= obj.x, y -= obj.y, z -= obj.z, *this;
  }
};

class ICM20602 {
private:
  static constexpr float ACCEL_G = 9806.65f; //< [mm/s/s]
#ifndef M_PI
  static constexpr float M_PI = 3.1415926535897932384626433832795f;
#endif
  static constexpr float ICM20602_ACCEL_FACTOR = 4 * 2048.0f;
  static constexpr float ICM20602_GYRO_FACTOR = 16.4f;

public:
  ICM20602() {}
  MotionParameter accel, gyro; //< ToDo: make private

public:
  bool init(spi_host_device_t spi_host, int8_t pin_cs) {
    // ESP-IDF SPI device initialization
    static spi_device_interface_config_t dev_cfg;
    dev_cfg.command_bits = 0;
    dev_cfg.address_bits = 8;
    dev_cfg.dummy_bits = 0;
    dev_cfg.mode = 3;
    dev_cfg.duty_cycle_pos = 0;
    dev_cfg.cs_ena_pretrans = 0;
    dev_cfg.cs_ena_posttrans = 0;
    dev_cfg.clock_speed_hz = 20'000'000;
    dev_cfg.spics_io_num = pin_cs;
    dev_cfg.flags = 0;
    dev_cfg.queue_size = 2;
    dev_cfg.pre_cb = NULL;
    dev_cfg.post_cb = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &spi_handle));
    return reset();
  }
  void device_reset() { writeReg(107, 0x81); /*< PowerManagement 1 */ }
  void device_config() {
    /* 107: Power Management 1
     *   7  : RESET
     *   6  : SLEEP
     *   5  : CYCLE
     *   4  : GYRO_STANDBY
     *   3  : TEMP_DIS
     *   2:0: CLKSEL
     */
    /* 26: CONFIG
     *   7  : -
     *   6  : FIFO_MODE
     *   5:3: EXT_SYNC_SET
     *   2:0: DLPF_CFG; see table1
     */
    /* 27: GYRO_CONFIG
     *  7:5: self-test
     *  4:3: FS_SEL; 0:250dps, 1:500dps, 2:1000dps, 3:2000dps
     *  2  : -
     *  1:0: FCHOISE_B; see table1
     */
    /* 28: ACCEL_CONFIG
     *  7:5: self-test
     *  4:3: ACCEL_FS_SEL; 0:2g, 1:4g, 2:8g, 3:16g
     *  2:0: -
     */
    /*
     * 29: ACCEL_CONFIG2
     * 5:4: DEC2_CFG; 0:4, 1:8, 2:16, 3:32 samples average
     * 3  : ACCEL_FCHOICE_B; 0:1[kHz], 1:4[kHz]
     * 2:0: A_DLPF_CFG; see table2
     */
    /* Gyro */
    writeReg(26, 0x00); /*< Config; 2:0==0: DLPF = 250[Hz] */
    writeReg(27, 0x18); /*< Gyr Conf; 4:3 FS=2000[dps], 1:0 FCHOICE=8[kHz] */
    /* Acceleration */
    // writeReg(28, 0x18); /*< Acc Conf; 4:3 FS=16[g] */
    writeReg(28, 0x08); /*< Acc Conf; 4:3 FS=2[g] */
    writeReg(29, 0x00); /*< Acc Conf 2; 3 F_CHOICE=1[kHz], DLPF=218.1[Hz] */
    // writeReg(17, 0xc9); /*< ??? for Accel */
    /* Power */
    writeReg(107, 0x01); /*< PowerManagement 1; unset SLEEP */
  }
  bool reset() {
    device_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
    device_config();
    return whoami();
  }
  bool whoami() {
    uint8_t whoami = readReg(117); /*< Who am I */
    uint8_t whoami_expected = 0x12;
    if (whoami != whoami_expected) {
      ESP_LOGE(TAG, "whoami failed:( 0x%X != 0x%X", whoami, whoami_expected);
      return false;
    }
    return true;
  }
  void update() {
    union {
      int16_t i;
      struct {
        uint8_t l : 8;
        uint8_t h : 8;
      };
    } bond;
    uint8_t rx[14];
    readReg(0x3b, rx, 14);
    bond.h = rx[0];
    bond.l = rx[1];
    accel.x = bond.i / ICM20602_ACCEL_FACTOR * ACCEL_G - accel_offset.x;
    bond.h = rx[2];
    bond.l = rx[3];
    accel.y = bond.i / ICM20602_ACCEL_FACTOR * ACCEL_G - accel_offset.y;
    bond.h = rx[4];
    bond.l = rx[5];
    accel.z = bond.i / ICM20602_ACCEL_FACTOR * ACCEL_G - accel_offset.z;

    bond.h = rx[8];
    bond.l = rx[9];
    gyro.x = bond.i / ICM20602_GYRO_FACTOR * M_PI / 180 - gyro_offset.x;
    bond.h = rx[10];
    bond.l = rx[11];
    gyro.y = bond.i / ICM20602_GYRO_FACTOR * M_PI / 180 - gyro_offset.y;
    bond.h = rx[12];
    bond.l = rx[13];
    gyro.z = bond.i / ICM20602_GYRO_FACTOR * M_PI / 180 - gyro_offset.z;
  }
  // void calibration() {
  //   const int ave_count = 100;
  //   for (int j = 0; j < 2; j++) {
  //     TickType_t xLastWakeTime = xTaskGetTickCount();
  //     MotionParameter accel_sum, gyro_sum;
  //     for (int i = 0; i < ave_count; i++) {
  //       vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  //       update();
  //       accel_sum += accel;
  //       gyro_sum += gyro;
  //     }
  //     accel_offset += accel_sum / ave_count;
  //     gyro_offset += gyro_sum / ave_count;
  //   }
  // }

private:
  static constexpr const char *TAG = "ICM-20602";
  spi_device_handle_t spi_handle = NULL;
  MotionParameter accel_offset, gyro_offset;

  void writeReg(uint8_t reg, uint8_t data) {
    static spi_transaction_t tx;
    tx.flags |= SPI_TRANS_USE_TXDATA;
    tx.addr = reg;
    tx.tx_data[0] = data;
    tx.length = 8;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
  }
  uint8_t readReg(const uint8_t reg) {
    static spi_transaction_t tx;
    tx.flags |= SPI_TRANS_USE_RXDATA;
    tx.addr = 0x80 | reg;
    tx.length = 8;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
    return tx.rx_data[0];
  }
  void readReg(const uint8_t reg, uint8_t *rx_buffer, size_t length) {
    static spi_transaction_t tx;
    tx.addr = 0x80 | reg;
    tx.tx_buffer = NULL;
    tx.rx_buffer = rx_buffer;
    tx.length = 8 * length;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
  }
};
