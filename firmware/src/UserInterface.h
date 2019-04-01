/**
 * @file UserInterface.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief マイクロマウスに備わっているセンサを用いてUIを構成する
 * @version 0.1
 * @date 2019-02-10
 *
 * @copyright Copyright (c) 2019
 *
 */
#pragma once

#include "app_log.h"
#include "config/io_mapping.h"
#include "config/model.h"
#include "global.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class UserInterface {
private:
  /* 定数 */
  static constexpr float thr_battery = 3.8f;
  /* UI パラメータ */
  static constexpr float thr_accel = 3 * 9807; /**< 加速度の閾値 */
  static constexpr float thr_gyro = 4 * PI;    /**< 角速度の閾値 */
  static constexpr float wait_ms = 200; /**< チャタリング防止時間 */
  static constexpr int thr_ref_front = 2400; /**< 前壁センサの閾値 */
  static constexpr int thr_ref_side = 2400;  /**< 横壁センサの閾値 */
  static constexpr float enc_interval_mm =
      10.0f; /**< エンコーダのカウント間隔 */
  static constexpr int wait_fix_ms = 1000; /**< 静止待機の最小静止時間 */
  static constexpr float thr_fix_gyro =
      0.01f * M_PI; /**< 静止待機の角速度の閾値 */

public:
  UserInterface() {}
  /**
   * @brief ユーザーに番号を選択させる
   *
   * @param range 番号の範囲．[0, range)
   * @return int 0 ~ range-1: 選択された番号
   * @return int -1: キャンセルされた
   */
  static int waitForSelect(int range = 16) {
    float prev_enc = enc.position(0) + enc.position(1);
    uint8_t value = 0;
    led = value;
    while (1) {
      float now_enc = enc.position(0) + enc.position(1);
      vTaskDelay(10 / portTICK_PERIOD_MS);
      /* SELECT */
      if (imu.gyro.y > thr_gyro) {
        value += range - 1;
        value %= range;
        led = value;
        bz.play(Buzzer::SELECT);
        vTaskDelay(wait_ms / portTICK_PERIOD_MS);
      }
      if (imu.gyro.y < -thr_gyro) {
        value += 1;
        value %= range;
        led = value;
        bz.play(Buzzer::SELECT);
        vTaskDelay(wait_ms / portTICK_PERIOD_MS);
      }
      if (now_enc > prev_enc + enc_interval_mm) {
        prev_enc += enc_interval_mm;
        value += 1;
        value %= range;
        led = value;
        bz.play(Buzzer::SELECT);
      }
      if (now_enc < prev_enc - enc_interval_mm) {
        prev_enc -= enc_interval_mm;
        value += range - 1;
        value %= range;
        led = value;
        bz.play(Buzzer::SELECT);
      }
      /* CONFIRM */
      if (std::abs(imu.accel.z) > thr_accel) {
        bz.play(Buzzer::CONFIRM);
        vTaskDelay(wait_ms / portTICK_PERIOD_MS);
        return value;
      }
      if (btn.pressed) {
        btn.flags = 0;
        bz.play(Buzzer::CONFIRM);
        return value;
      }
      /* CANCEL */
      if (std::abs(imu.accel.x) > thr_accel) {
        bz.play(Buzzer::CANCEL);
        vTaskDelay(wait_ms / portTICK_PERIOD_MS);
        return -1;
      }
      if (btn.long_pressed_1) {
        btn.flags = 0;
        bz.play(Buzzer::CANCEL);
        return -1;
      }
    }
    /* it will not reach here */
    return -1;
  }
  /**
   * @brief 前壁センサが遮られるのを待つ関数
   *
   * @param side センサ位置 true: side, false: front
   * @return true 遮られた
   * @return false キャンセルされた
   */
  static bool waitForCover(bool side = false) {
    while (1) {
      vTaskDelay(wait_ms / portTICK_PERIOD_MS);
      /* CONFIRM */
      if (!side && ref.front(0) > thr_ref_front &&
          ref.front(1) > thr_ref_front) {
        bz.play(Buzzer::CONFIRM);
        return true;
      }
      if (side && ref.side(0) > thr_ref_side && ref.side(1) > thr_ref_side) {
        bz.play(Buzzer::CONFIRM);
        return true;
      }
      /* CANCEL */
      if (std::abs(imu.accel.x) > thr_accel) {
        bz.play(Buzzer::CANCEL);
        vTaskDelay(wait_ms / portTICK_PERIOD_MS);
        return false;
      }
      if (btn.long_pressed_1) {
        btn.flags = 0;
        bz.play(Buzzer::CANCEL);
        return false;
      }
    }
  }
  /**
   * @brief 静止状態になるまで待機する関数
   *
   * @return true 静止状態になった
   * @return false キャンセルされた
   */
  static bool waitForFix() {
    int fix_count = 0;
    while (1) {
      vTaskDelay(1 / portTICK_PERIOD_MS);
      /* FIX */
      if (std::abs(imu.gyro.x) < thr_fix_gyro &&
          std::abs(imu.gyro.y) < thr_fix_gyro &&
          std::abs(imu.gyro.z) < thr_fix_gyro) {
        if (fix_count++ > wait_fix_ms) {
          bz.play(Buzzer::CONFIRM);
          return true;
        }
      } else {
        fix_count = 0;
      }
      /* CANCEL */
      if (btn.pressed) {
        btn.flags = 0;
        bz.play(Buzzer::CANCEL);
        return false;
      }
      if (std::abs(imu.accel.x) > thr_accel) {
        bz.play(Buzzer::CANCEL);
        vTaskDelay(wait_ms / portTICK_PERIOD_MS);
        return false;
      }
    }
  }
  /**
   * @brief Sample the Battery Voltage
   * @return float voltage [V]
   */
  static float getBatteryVoltage() {
    return 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
    // return 2 * 1.0f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
  }
  /**
   * @brief バッテリー電圧をLEDで表示
   * @param voltage [V]
   */
  static void batteryLedIndicate(const float voltage) {
    led = 0;
    if (voltage < 4.0f)
      led = 0x01;
    else if (voltage < 4.1f)
      led = 0x03;
    else if (voltage < 4.2f)
      led = 0x07;
    else
      led = 0x0F;
  }
  static void batteryCheck() {
    float voltage = getBatteryVoltage();
    batteryLedIndicate(voltage);
    logi << "Battery Voltage: " << voltage << std::endl;
    if (voltage < thr_battery) {
      logw << "Battery Low!" << std::endl;
      bz.play(Buzzer::LOW_BATTERY);
      while (!btn.pressed)
        vTaskDelay(100 / portTICK_PERIOD_MS);
      btn.flags = 0;
      led = 0;
    }
  }
  static void gointToDeepsleep() {
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
    vTaskDelay(portMAX_DELAY);
  }
};
