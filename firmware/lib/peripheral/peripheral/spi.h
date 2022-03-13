/**
 * @file spi.h
 * @brief SPI utility
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-05-23
 * @copyright Copyright 2020 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <driver/spi_master.h>

namespace peripheral {

class SPI {
 public:
  static bool install(spi_host_device_t spi_host,
                      gpio_num_t pin_sclk,
                      gpio_num_t pin_miso,
                      gpio_num_t pin_mosi,
                      int dma_chain) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = pin_mosi,
        .miso_io_num = pin_miso,
        .sclk_io_num = pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        // .data2_io_num = -1,
        // .data3_io_num = -1,
        // .data4_io_num = -1,
        // .data5_io_num = -1,
        // .data6_io_num = -1,
        // .data7_io_num = -1,
        .max_transfer_sz = 0,
        .flags = 0,
        .intr_flags = 0,
    };
    esp_err_t ret = spi_bus_initialize(spi_host, &bus_cfg, dma_chain);
    if (ret != ESP_OK) {
      ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
      return false;
    }
    return true;
  }
};

};  // namespace peripheral
