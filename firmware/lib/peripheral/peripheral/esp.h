#pragma once

#include <esp_system.h>

namespace peripheral {

class ESP {
public:
  static uint64_t get_mac() {
    uint64_t mac = 0;
    esp_efuse_mac_get_default((uint8_t *)&mac);
    return mac;
  }
};

}; // namespace peripheral
