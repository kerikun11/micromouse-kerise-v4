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
    dev_cfg.clock_speed_hz = 10'000'000;
    dev_cfg.spics_io_num = pin_cs;
    dev_cfg.flags = 0;
    dev_cfg.queue_size = 1;
    dev_cfg.pre_cb = NULL;
    dev_cfg.post_cb = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &encoder_spi));
    return true;
  }
  bool update() {
    uint8_t rx_buf[4];
    static spi_transaction_t tx;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.tx_data[0] = 0xFF;
    tx.tx_data[1] = 0xFF;
    tx.tx_data[2] = 0xFF;
    tx.tx_data[3] = 0xFF;
    tx.rx_buffer = rx_buf;
    tx.length = 32;
    ESP_ERROR_CHECK(spi_device_transmit(encoder_spi, &tx));

    pulses[0] = (uint16_t(0x3F & rx_buf[0]) << 8) | rx_buf[1];
    pulses[1] = (uint16_t(0x3F & rx_buf[2]) << 8) | rx_buf[3];

    return true;
  }
  int get(int ch) const { return pulses[ch]; }

private:
  spi_device_handle_t encoder_spi = NULL;
  int pulses[2];
};
