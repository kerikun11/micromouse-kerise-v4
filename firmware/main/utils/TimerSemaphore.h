#pragma once

#include <Arduino.h>

class TimerSemaphore {
  public:
    TimerSemaphore() : esp_timer_handle(nullptr) {
      SemaphoreHandle = xSemaphoreCreateBinary();
    }
    void periodic(uint32_t us) {
      attach(us, true, [](void* arg) {
        static_cast<TimerSemaphore*>(arg)->give();
      }, this);
    }
    void once(uint32_t us) {
      attach(us, false, [](void* arg) {
        static_cast<TimerSemaphore*>(arg)->give();
      }, this);
    }
    void end() {
      detach();
    }
    portBASE_TYPE take(portTickType xBlockTime = portMAX_DELAY) {
      return xSemaphoreTake(SemaphoreHandle, xBlockTime);
    }

  private:
    esp_timer_handle_t esp_timer_handle;
    volatile SemaphoreHandle_t SemaphoreHandle;

    portBASE_TYPE give() {
      return xSemaphoreGiveFromISR(SemaphoreHandle, NULL);
    }
    void attach(uint32_t microseconds, bool repeat, esp_timer_cb_t callback, void* arg) {
      if (esp_timer_handle) {
        esp_timer_stop(esp_timer_handle);
        esp_timer_delete(esp_timer_handle);
      }
      esp_timer_create_args_t esp_timer_create_args;
      esp_timer_create_args.arg = arg;
      esp_timer_create_args.callback = callback;
      esp_timer_create_args.dispatch_method = ESP_TIMER_TASK;
      esp_timer_create_args.name = "Ticker";
      ESP_ERROR_CHECK(esp_timer_create(&esp_timer_create_args, &esp_timer_handle));
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

