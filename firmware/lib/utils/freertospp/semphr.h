/**
 * @file semphr.h
 * @brief C++ Wrapper for FreeRTOS in ESP32
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2018-07-09
 */
#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace freertospp {

/**
 * @brief C++ Wrapper for Semaphore function
 */
class Semaphore {
 public:
  Semaphore() {
    xSemaphore = xSemaphoreCreateBinary();
    if (xSemaphore == NULL) {
      ESP_LOGE(TAG, "xSemaphoreCreateBinary() failed");
    }
  }
  ~Semaphore() { vSemaphoreDelete(xSemaphore); }
  bool giveFromISR() const {
    return pdTRUE == xSemaphoreGiveFromISR(xSemaphore, NULL);
  }
  bool give() const { return pdTRUE == xSemaphoreGive(xSemaphore); }
  bool take(portTickType xBlockTime = portMAX_DELAY) const {
    return pdTRUE == xSemaphoreTake(xSemaphore, xBlockTime);
  }

 private:
  static constexpr const char* TAG = "Semaphore";
  SemaphoreHandle_t xSemaphore = NULL;
};

}  // namespace freertospp
