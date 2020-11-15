/**
 * @file ma730.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief MA730 Magnetic Encoder Driver
 * @date 2020-05-23
 * @copyright Copyright (c) 2020 Ryotaro Onuki
 */
#pragma once

#include <driver/spi_master.h>
#include <esp_err.h>

class MA730 {
public:
  static constexpr int PULSES_SIZE = 16384;

public:
  MA730() {}
  bool init(spi_host_device_t spi_host, gpio_num_t pin_cs) {
    // ESP-IDF SPI device initialization
    static spi_device_interface_config_t dev_cfg;
    dev_cfg.command_bits = 0;
    dev_cfg.address_bits = 0;
    dev_cfg.dummy_bits = 0;
    dev_cfg.mode = 3;
    dev_cfg.duty_cycle_pos = 0;
    dev_cfg.cs_ena_pretrans = 0;
    dev_cfg.cs_ena_posttrans = 0;
    dev_cfg.clock_speed_hz = 10 * 1000 * 1000;
    dev_cfg.input_delay_ns = 0;
    dev_cfg.spics_io_num = pin_cs;
    dev_cfg.flags = 0;
    dev_cfg.queue_size = 1;
    dev_cfg.pre_cb = NULL;
    dev_cfg.post_cb = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &encoder_spi));
    return true;
  }
  void update() {
    uint8_t rx_buf[2];
    static spi_transaction_t tx;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.user = this;
    tx.tx_data[0] = 0x00;
    tx.tx_data[1] = 0x00;
    tx.rx_buffer = rx_buf;
    tx.length = 16;
    ESP_ERROR_CHECK(spi_device_transmit(encoder_spi, &tx));

    pulses = uint16_t(rx_buf[0] << 6) | (rx_buf[1] >> 2);
  }
  int get() const { return pulses; }

private:
  spi_device_handle_t encoder_spi = NULL;
  int pulses;
};
