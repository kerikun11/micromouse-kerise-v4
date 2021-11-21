/**
 * @file TimerSemaphore.h
 * @brief Timer Semaphore for ESP32
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include <esp_timer.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/semphr.h>

class TimerSemaphore {
public:
  TimerSemaphore() : esp_timer_handle(nullptr) {
    xSemaphore = xSemaphoreCreateBinary();
  }
  ~TimerSemaphore() {
    end();
    vSemaphoreDelete(xSemaphore);
  }
  void periodic(uint32_t microseconds) {
    attach(
        microseconds, true,
        [](void *arg) { static_cast<decltype(this)>(arg)->giveFromISR(); },
        this);
  }
  void oneshot(uint32_t microseconds) {
    attach(
        microseconds, false,
        [](void *arg) { static_cast<decltype(this)>(arg)->giveFromISR(); },
        this);
  }
  void end() { detach(); }
  portBASE_TYPE take(TickType_t xBlockTime = portMAX_DELAY) {
    return xSemaphoreTake(xSemaphore, xBlockTime);
  }

private:
  SemaphoreHandle_t xSemaphore;
  esp_timer_handle_t esp_timer_handle;

  portBASE_TYPE giveFromISR() {
    return xSemaphoreGiveFromISR(xSemaphore, NULL);
  }
  void attach(uint32_t microseconds, bool repeat, esp_timer_cb_t callback,
              void *arg) {
    detach();
    esp_timer_create_args_t esp_timer_create_args;
    esp_timer_create_args.arg = arg;
    esp_timer_create_args.callback = callback;
    esp_timer_create_args.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create_args.name = "TimerSemaphore";
    ESP_ERROR_CHECK(
        esp_timer_create(&esp_timer_create_args, &esp_timer_handle));
    if (repeat) {
      ESP_ERROR_CHECK(esp_timer_start_periodic(esp_timer_handle, microseconds));
    } else {
      ESP_ERROR_CHECK(esp_timer_start_once(esp_timer_handle, microseconds));
    }
  }
  void detach() {
    if (esp_timer_handle) {
      esp_timer_stop(esp_timer_handle);
      esp_timer_delete(esp_timer_handle);
      esp_timer_handle = nullptr;
    }
  }
};
