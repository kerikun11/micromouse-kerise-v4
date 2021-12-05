/**
 * @file taskbase.h
 * @brief C++ Wrapper for FreeRTOS in ESP32
 * @author Ryotaro Onuki
 * @date 2018-07-09
 */
#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace freertospp {

/**
 * @brief FreeRTOSのタスクのベースとなるクラス．
 * 実行したい関数をもつクラスは，このクラスを継承する．
 */
class TaskBase {
public:
  /**
   * @brief Construct a new Task Base object
   * このコンストラクタを呼ぶことはない
   */
  TaskBase() : pxCreatedTask(NULL) {}
  /**
   * @brief Destroy the Task Base object
   * もしタスクが実行中なら削除する
   */
  ~TaskBase() { deleteTask(); }
  /**
   * @brief Create a Task object
   *
   * @param pcName
   * @param uxPriority
   * @param usStackDepth
   * @param xCoreID
   * @return true
   * @return false
   */
  bool createTask(const char *pcName, UBaseType_t uxPriority = 0,
                  const uint16_t usStackDepth = configMINIMAL_STACK_SIZE,
                  const BaseType_t xCoreID = tskNO_AFFINITY) {
    if (pxCreatedTask != NULL) {
      ESP_LOGW(TAG, "task \"%s\" is already created", pcName);
      return false;
    }
    BaseType_t res =
        xTaskCreatePinnedToCore(pxTaskCode, pcName, usStackDepth, this,
                                uxPriority, &pxCreatedTask, xCoreID);
    if (res != pdPASS) {
      ESP_LOGW(TAG, "couldn't create the task \"%s\"", pcName);
      return false;
    }
    return true;
  }
  /**
   * @brief タスクを削除する関数
   */
  void deleteTask() {
    if (pxCreatedTask == NULL) {
      ESP_LOGW(TAG, "task is not created");
      return;
    }
    vTaskDelete(pxCreatedTask);
    pxCreatedTask = NULL;
  }

protected:
  static constexpr const char *TAG = "TaskBase";
  TaskHandle_t pxCreatedTask; //< タスクのハンドル

  /**
   * @brief FreeRTOS
   * により実行される関数の宣言．実体は継承クラスで定義すること．
   */
  virtual void task() = 0;
  /**
   * @brief FreeRTOS により実行される静的関数ポインタ
   * @param pvParameters this ポインタ
   */
  static void pxTaskCode(void *const pvParameters) {
    static_cast<TaskBase *>(pvParameters)->task();
  }
};

} // namespace freertospp
