/**
 * @file mutex.h
 * @brief C++ Wrapper for FreeRTOS in ESP32
 * @author Ryotaro Onuki
 * @date 2018-07-09
 */
#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace freertospp {

/**
 * @brief C++ Wrapper for Mutex function
 */
class Mutex {
public:
  Mutex() {
    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore == NULL) {
      ESP_LOGE(tag, "xSemaphoreCreateMutex() failed");
    }
  }
  ~Mutex() { vSemaphoreDelete(xSemaphore); }
  bool giveFromISR() {
    return pdTRUE == xSemaphoreGiveFromISR(xSemaphore, NULL);
  }
  bool give() { return pdTRUE == xSemaphoreGive(xSemaphore); }
  bool take(portTickType xBlockTime = portMAX_DELAY) {
    return pdTRUE == xSemaphoreTake(xSemaphore, xBlockTime);
  }

private:
  const char *tag = "Mutex";
  SemaphoreHandle_t xSemaphore = NULL;
};

} // namespace freertospp
