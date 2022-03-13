/**
 * @file esp.h
 * @brief ESP32 utility
 * @author Ryotaro Onuki <kerikun11+github@gmail.com> <kerikun11+github@gmail.com>
 * @date 2022-03-13
 * @copyright Copyright 2022 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <esp_system.h>

namespace peripheral {

class ESP {
 public:
  static uint64_t get_mac() {
    uint64_t mac = 0;
    esp_efuse_mac_get_default((uint8_t*)&mac);
    return mac;
  }
};

};  // namespace peripheral
