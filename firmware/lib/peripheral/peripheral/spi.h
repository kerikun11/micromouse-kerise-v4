/**
 * @file spi.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief SPI utility
 * @date 2020-05-23
 * @copyright Copyright (c) 2020 Ryotaro Onuki
 */
#pragma once

#include <driver/spi_master.h>
#include <esp_err.h>

class SPI {
public:
  static bool install(spi_host_device_t spi_host, gpio_num_t pin_sclk,
                      gpio_num_t pin_miso, gpio_num_t pin_mosi, int dma_chain) {
    static spi_bus_config_t bus_cfg;
    bus_cfg.mosi_io_num = pin_mosi;
    bus_cfg.miso_io_num = pin_miso;
    bus_cfg.sclk_io_num = pin_sclk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 0;
    esp_err_t ret = spi_bus_initialize(spi_host, &bus_cfg, dma_chain);
    if (ret != ESP_OK)
      std::cerr << "[E][" __FILE__ ":" << __LINE__ << "] "
                << esp_err_to_name(ret) << std::endl;
    return ret == ESP_OK;
  }
};
