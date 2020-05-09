#pragma once

#include <driver/spi_master.h>
#include <esp_err.h>

class MA730 {
public:
  static constexpr int MA730_PULSES = 16384;
  static constexpr int PULSES_SIZE = 16384;
#ifndef M_PI
  static constexpr float M_PI = 3.1415926535897932384626433832795f;
#endif

public:
  MA730() {}
  bool begin(spi_host_device_t spi_host, gpio_num_t pin_cs) {
    this->pin_cs = pin_cs;
    gpio_set_direction(pin_cs, GPIO_MODE_OUTPUT);
    gpio_set_level(pin_cs, 1);
    // ESP-IDF SPI device initialization
    static spi_device_interface_config_t dev_cfg = {0};
    dev_cfg.command_bits = 0;
    dev_cfg.address_bits = 0;
    dev_cfg.dummy_bits = 0;
    dev_cfg.mode = 3;
    dev_cfg.duty_cycle_pos = 0;
    dev_cfg.cs_ena_pretrans = 0;
    dev_cfg.cs_ena_posttrans = 0;
    dev_cfg.clock_speed_hz = 20000000;
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
    uint8_t rxbuf[4];
    static spi_transaction_t tx;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.user = this;
    tx.tx_data[0] = 0x00;
    tx.tx_data[1] = 0x00;
    tx.rx_buffer = rxbuf;
    tx.length = 16;
    ESP_ERROR_CHECK(spi_device_transmit(encoder_spi, &tx)); //< send command
    ESP_ERROR_CHECK(spi_device_transmit(encoder_spi, &tx)); //< read data

    pulses = ((uint16_t)rxbuf[0] << 6) | (rxbuf[1] >> 2);
  }
  int get() const { return pulses; }

private:
  spi_device_handle_t encoder_spi = NULL;
  gpio_num_t pin_cs;
  int pulses;
};
