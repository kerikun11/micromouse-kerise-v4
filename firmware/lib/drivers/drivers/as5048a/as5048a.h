/**
 * @file as5048a.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief Driver of Dual Chain-Connected Magnetic Encoder AS5048A_DUAL
 * @date 2020-05-09
 * @copyright Copyright (c) 2020 Ryotaro Onuki
 */
#pragma once

#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_log.h>

class AS5048A_DUAL {
public:
  static constexpr int PULSES_SIZE = 16384;

public:
  AS5048A_DUAL() {}
  bool init(const spi_host_device_t spi_host, const int8_t pin_cs) {
    // ESP-IDF SPI device initialization
    static spi_device_interface_config_t dev_cfg;
    dev_cfg.command_bits = 1;
    dev_cfg.address_bits = 0;
    dev_cfg.dummy_bits = 0;
    dev_cfg.mode = 1;
    dev_cfg.duty_cycle_pos = 0;
    dev_cfg.cs_ena_pretrans = 0;
    dev_cfg.cs_ena_posttrans = 0;
    dev_cfg.clock_speed_hz = 5'000'000;
    dev_cfg.spics_io_num = pin_cs;
    dev_cfg.flags = 0;
    dev_cfg.queue_size = 1;
    dev_cfg.pre_cb = NULL;
    dev_cfg.post_cb = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &encoder_spi));
    return update();
  }
  bool update() {
    bool res = true;
    /* transaction */
    static spi_transaction_t tx;
    tx.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    tx.tx_data[0] = tx.tx_data[1] = tx.tx_data[2] = tx.tx_data[3] = 0xFF;
    tx.length = 32;
    ESP_ERROR_CHECK(spi_device_transmit(encoder_spi, &tx));
    /* data parse */
    uint16_t pkt[2];
    pkt[0] = (tx.rx_data[0] << 8) | tx.rx_data[1];
    pkt[1] = (tx.rx_data[2] << 8) | tx.rx_data[3];
    for (int i = 0; i < 2; i++) {
      if (calc_even_parity(pkt[i])) {
        ESP_LOGW(TAG, "parity error. pkt[%d]: 0x%04X", i, (int)pkt[i]);
        res = false;
        continue;
      }
      pulses[i] = pkt[i] & 0x3FFF;
    }
    return res;
  }
  int get(int ch) const { return pulses[ch]; }

private:
  static constexpr const char *TAG = "AS5048A";
  spi_device_handle_t encoder_spi = NULL;
  int pulses[2] = {0, 0};

  static uint8_t calc_even_parity(uint16_t data) {
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return data & 1;
  }
};
