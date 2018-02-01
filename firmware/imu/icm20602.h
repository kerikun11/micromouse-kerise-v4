#pragma once

#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_err.h"

struct MotionParameter {
  float x, y, z;
  MotionParameter(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
  MotionParameter operator+(const MotionParameter& obj) const {
    return MotionParameter(x + obj.x, y + obj.y, z + obj.z);
  }
  MotionParameter operator*(const float mul) const {
    return MotionParameter(x * mul, y * mul, z * mul);
  }
  MotionParameter operator/(const float div) const {
    return MotionParameter(x / div, y / div, z / div);
  }
  const MotionParameter& operator=(const MotionParameter& obj) {
    x = obj.x; y = obj.y; z = obj.z; return *this;
  }
  const MotionParameter& operator+=(const MotionParameter& obj) {
    x += obj.x; y += obj.y; z += obj.z; return *this;
  }
  const MotionParameter& operator/=(const float& div) {
    x /= div; y /= div; z /= div; return *this;
  }
};

#define ICM20602_ACCEL_FACTOR 2048.0f
#define ICM20602_GYRO_FACTOR  16.4f
#define ICM20602_ACCEL_G      9806.65f

class ICM20602 {
  public:
    ICM20602() {}
    MotionParameter accel, gyro;

  public:
    bool begin(spi_host_device_t spi_host, int8_t pin_cs) {
      // ESP-IDF SPI device initialization
      spi_device_interface_config_t dev_cfg = {0};
      dev_cfg.command_bits = 0;         ///< Default amount of bits in command phase (0-16), used when ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored.
      dev_cfg.address_bits = 8;         ///< Default amount of bits in address phase (0-64), used when ``SPI_TRANS_VARIABLE_ADDR`` is not used, otherwise ignored.
      dev_cfg.dummy_bits = 0;           ///< Amount of dummy bits to insert between address and data phase
      dev_cfg.mode = 3;                 ///< SPI mode (0-3)
      dev_cfg.duty_cycle_pos = 0;       ///< Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
      dev_cfg.cs_ena_pretrans = 0;      ///< Amount of SPI bit-cycles the cs should be activated before the transmission (0-16). This only works on half-duplex transactions.
      dev_cfg.cs_ena_posttrans = 0;     ///< Amount of SPI bit-cycles the cs should stay active after the transmission (0-16)
      dev_cfg.clock_speed_hz = 10000000;///< Clock speed, in Hz
      dev_cfg.spics_io_num = pin_cs;    ///< CS GPIO pin for this device, or -1 if not used
      dev_cfg.flags = 0;                ///< Bitwise OR of SPI_DEVICE_* flags
      dev_cfg.queue_size = 1;           ///< Transaction queue size. This sets how many transactions can be 'in the air' (queued using spi_device_queue_trans but not yet finished using spi_device_get_trans_result) at the same time
      dev_cfg.pre_cb = NULL;            ///< Callback to be called before a transmission is started. This callback is called within interrupt context.
      dev_cfg.post_cb = NULL;           ///< Callback to be called after a transmission has completed. This callback is called within interrupt context.
      ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &spi_handle));
      return reset();
    }
    bool reset() {
      writeReg(0x6b, 0x01); //< power management 1
      writeReg(0x1b, 0x18); //< gyro range
      writeReg(0x1c, 0x18); //< accel range
      return whoami();
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
      bond.h = rx[0]; bond.l = rx[1];
      accel.x = bond.i / ICM20602_ACCEL_FACTOR * ICM20602_ACCEL_G - accel_offset.x;
      bond.h = rx[2]; bond.l = rx[3];
      accel.y = bond.i / ICM20602_ACCEL_FACTOR * ICM20602_ACCEL_G - accel_offset.y;
      bond.h = rx[4]; bond.l = rx[5];
      accel.z = bond.i / ICM20602_ACCEL_FACTOR * ICM20602_ACCEL_G - accel_offset.z;

      bond.h = rx[8]; bond.l = rx[9];
      gyro.x = bond.i / ICM20602_GYRO_FACTOR * PI / 180 - gyro_offset.x;
      bond.h = rx[10]; bond.l = rx[11];
      gyro.y = bond.i / ICM20602_GYRO_FACTOR * PI / 180 - gyro_offset.y;
      bond.h = rx[12]; bond.l = rx[13];
      gyro.z = bond.i / ICM20602_GYRO_FACTOR * PI / 180 - gyro_offset.z;
    }
    void calibration() {
      MotionParameter accel_sum, gyro_sum;
      const int ave_count = 500;
      for (int i = 0; i < 2 ; i++) {
        portTickType xLastWakeTime = xTaskGetTickCount();
        for (int i = 0; i < ave_count; i++) {
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
          update();
          accel_sum += accel;
          gyro_sum += gyro;
        }
        accel_offset += accel_sum / ave_count;
        gyro_offset += gyro_sum / ave_count;
      }
    }

  private:
    spi_device_handle_t spi_handle;
    MotionParameter accel_offset, gyro_offset;

    bool whoami() {
      if (readReg(117) != 0x12) {
        log_e("whoami failed:(");
        return false;
      }
      return true;
    }
    void writeReg(uint8_t reg, uint8_t data) {
      static spi_transaction_t tx = {0};
      tx.flags |= SPI_TRANS_USE_TXDATA;
      tx.addr = reg;
      tx.tx_data[0] = data;
      tx.length = 8;
      ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
    }
    uint8_t readReg(const uint8_t reg) {
      static spi_transaction_t tx = {0};
      tx.flags |= SPI_TRANS_USE_RXDATA;
      tx.addr = 0x80 | reg;
      tx.length = 8;
      ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
      return tx.rx_data[0];
    }
    void readReg(const uint8_t reg, uint8_t *rx_buffer, size_t length) {
      static spi_transaction_t tx = {0};
      tx.addr = 0x80 | reg;
      tx.tx_buffer = NULL;
      tx.rx_buffer = rx_buffer;
      tx.length = 8 * length;
      ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
    }
};

