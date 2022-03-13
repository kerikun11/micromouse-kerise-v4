/**
 * @file task.h
 * @brief C++ Wrapper for FreeRTOS in ESP32
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2018-07-09
 */
#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief C++ Wrapper for FreeRTOS API
 */
namespace freertospp {

/**
 * @brief C++ のメンバ関数を実行することができるタスクのクラス
 *
 * @tparam T 実行するメンバ関数のクラス
 */
template <typename T>
class Task {
 public:
  /**
   * @brief Construct a new Task object
   */
  Task() : pxCreatedTask(NULL) {}
  /**
   * @brief Destroy the Task object
   */
  ~Task() { terminate(); }
  /**
   * @brief タスクを生成し，実行開始する関数
   *
   * @param obj this ポインタ
   * @param func メンバ関数ポインタ，`&T::func` のように渡す
   * @param pcName タスク名文字列
   * @param usStackDepth スタックサイズ
   * @param uxPriority 優先度
   * @param xCoreID 実行させるCPUコア番号
   */
  bool start(T* obj,                    //< thisポインタ
             void (T::*func)(),         //< メンバ関数ポインタ
             const char* const pcName,  //< タスク名文字列
             unsigned short usStackDepth =
                 configMINIMAL_STACK_SIZE,           //< スタックサイズ
             unsigned portBASE_TYPE uxPriority = 0,  //< タスク優先度
             const BaseType_t xCoreID = tskNO_AFFINITY  //< 実行コア
  ) {
    this->obj = obj;
    this->func = func;
    if (pxCreatedTask != NULL) {
      ESP_LOGW(TAG, "task %s is already created", pcName);
      return false;
    }
    BaseType_t result =
        xTaskCreatePinnedToCore(entry_point, pcName, usStackDepth, this,
                                uxPriority, &pxCreatedTask, xCoreID);
    return result == pdPASS;
  }
  /**
   * @brief タスクを終了し，削除する関数
   */
  void terminate() {
    if (pxCreatedTask == NULL)
      return;
    vTaskDelete(pxCreatedTask);
    pxCreatedTask = NULL;
  }

 private:
  static constexpr const char* TAG = "Task";
  TaskHandle_t pxCreatedTask = NULL;  //< タスクのハンドル
  T* obj = NULL;                      //< thisポインタ
  void (T::*func)() = NULL;           //< メンバ関数ポインタ

  /**
   * @brief FreeRTOSにより実行される関数ポインタ
   */
  static void entry_point(void* arg) {
    auto task_obj = static_cast<Task*>(arg);
    (task_obj->obj->*task_obj->func)();
  }
};

}  // namespace freertospp
