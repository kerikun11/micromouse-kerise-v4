/**
 * @file thread.h
 * @brief C++ Wrapper for FreeRTOS in ESP32
 * @author Ryotaro Onuki
 * @date 2018-07-09
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <functional>

namespace freertospp {

/**
 * @brief C++ Wrapper for Thread function
 */
class Thread {
public:
  Thread(std::function<void()> func, const char *const pcName = "unknown",
         unsigned short usStackDepth = 8192,
         unsigned portBASE_TYPE uxPriority = 0,
         const BaseType_t xCoreID = tskNO_AFFINITY)
      : func(func) {
    xSemaphore = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(entry_point, pcName, usStackDepth, this, uxPriority,
                            &pxCreatedTask, xCoreID);
  }
  ~Thread() { detach(); }
  bool joinable() const { return pxCreatedTask != NULL; }
  void join(TickType_t xBlockTime = portMAX_DELAY) {
    xSemaphoreTake(xSemaphore, xBlockTime);
  }
  void detach() {
    if (pxCreatedTask == NULL)
      return;
    vTaskDelete(pxCreatedTask);
    pxCreatedTask = NULL;
    xSemaphoreGive(xSemaphore);
  }

private:
  TaskHandle_t pxCreatedTask = NULL;
  SemaphoreHandle_t xSemaphore = NULL;
  std::function<void()> func;

  static void entry_point(void *arg) {
    auto obj = static_cast<Thread *>(arg);
    obj->func();
    obj->detach();
  }
};

}; // namespace freertospp
