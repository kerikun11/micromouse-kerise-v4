#pragma once

#include <esp_err.h>
#include <esp_spiffs.h>
#include <iostream>

namespace peripheral {

class SPIFFS {
public:
  static bool init(const char *mount_path = "/spiffs") {
    esp_vfs_spiffs_conf_t conf = {.base_path = mount_path,
                                  .partition_label = NULL,
                                  .max_files = 5,
                                  .format_if_mount_failed = true};

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
      if (ret == ESP_FAIL) {
        std::cerr << "[E][" __FILE__ ":" << __LINE__ << "] " << //
            "Failed to mount or format filesystem" << std::endl;
      } else if (ret == ESP_ERR_NOT_FOUND) {
        std::cerr << "[E][" __FILE__ ":" << __LINE__ << "] " << //
            "Failed to find SPIFFS partition" << std::endl;
      } else {
        std::cerr << "[E][" __FILE__ ":" << __LINE__ << "] " << //
            "Failed to initialize SPIFFS (" << esp_err_to_name(ret) << ")"
                  << std::endl;
      }
      return false;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
      std::cerr << "[E][" __FILE__ ":" << __LINE__ << "] " << //
          "Failed to get SPIFFS partition information (" << esp_err_to_name(ret)
                << ")" << std::endl;
    } else {
      std::cerr << "[I][" __FILE__ ":" << __LINE__ << "] " << //
          "Partition size: total: " << total << ", used: " << used << std::endl;
    }

    return true;
  }
  static bool deinit() {
    esp_vfs_spiffs_unregister(NULL);
    return true;
  }
};

}; // namespace peripheral
