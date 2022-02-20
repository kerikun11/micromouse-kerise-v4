/**
 * @file UserInterface.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief マイクロマウスに備わっているセンサを用いてUIを構成する
 * @date 2019-02-10
 * @copyright Copyright (c) 2019 Ryotaro Onuki
 */
#pragma once

#include "app_log.h"
#include "config/io_mapping.h"
#include "config/model.h"
#include "hardware/hardware.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <peripheral/adc.h>

class UserInterface {
public:
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
      0.01f * PI; /**< 静止待機の角速度の閾値 */
  static constexpr float thr_gyro_pickup = PI; /**< 回収角速度の閾値 */

private:
  hardware::Hardware *hw;

public:
  UserInterface(hardware::Hardware *hw) : hw(hw) {}
  /**
   * @brief ユーザーに番号を選択させる
   *
   * @param range 番号の範囲．[1, 16]
   * @return int 0 ~ range-1: 選択された番号
   * @return int -1: キャンセルされた
   */
  int waitForSelect(const int range = 16, const uint8_t init_value = 0) {
    float prev_enc = hw->enc->get_position(0) + hw->enc->get_position(1);
    uint8_t value = init_value;
    hw->led->set(value);
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1));
      float now_enc = hw->enc->get_position(0) + hw->enc->get_position(1);
      /* SELECT */
      if (hw->imu->get_gyro().y > thr_gyro) {
        value += range - 1;
        value %= range;
        hw->led->set(value);
        hw->bz->play(hardware::Buzzer::SELECT);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
      }
      if (hw->imu->get_gyro().y < -thr_gyro) {
        value += 1;
        value %= range;
        hw->led->set(value);
        hw->bz->play(hardware::Buzzer::SELECT);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
      }
      if (now_enc > prev_enc + enc_interval_mm) {
        prev_enc += enc_interval_mm;
        value += 1;
        value %= range;
        hw->led->set(value);
        hw->bz->play(hardware::Buzzer::SELECT);
      }
      if (now_enc < prev_enc - enc_interval_mm) {
        prev_enc -= enc_interval_mm;
        value += range - 1;
        value %= range;
        hw->led->set(value);
        hw->bz->play(hardware::Buzzer::SELECT);
      }
      /* CONFIRM */
      if (std::abs(hw->imu->get_accel().z) > thr_accel) {
        hw->bz->play(hardware::Buzzer::CONFIRM);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        return value;
      }
      if (hw->btn->pressed) {
        hw->btn->flags = 0;
        hw->bz->play(hardware::Buzzer::CONFIRM);
        return value;
      }
      /* CANCEL */
      if (std::abs(hw->imu->get_accel().x) > thr_accel) {
        hw->bz->play(hardware::Buzzer::CANCEL);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        return -1;
      }
      if (hw->btn->long_pressed_1) {
        hw->btn->flags = 0;
        hw->bz->play(hardware::Buzzer::CANCEL);
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
  bool waitForCover(bool side = false) {
    if (side)
      hw->led->set(9);
    else
      hw->led->set(6);
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1));
      /* CONFIRM */
      if (!side && hw->rfl->front(0) > thr_ref_front &&
          hw->rfl->front(1) > thr_ref_front) {
        hw->bz->play(hardware::Buzzer::CONFIRM);
        return true;
      }
      if (side && hw->rfl->side(0) > thr_ref_side &&
          hw->rfl->side(1) > thr_ref_side) {
        hw->bz->play(hardware::Buzzer::CONFIRM);
        return true;
      }
      /* CANCEL */
      if (std::abs(hw->imu->get_accel().x) > thr_accel) {
        hw->bz->play(hardware::Buzzer::CANCEL);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        return false;
      }
      if (hw->btn->long_pressed_1) {
        hw->btn->flags = 0;
        hw->bz->play(hardware::Buzzer::CANCEL);
        return false;
      }
    }
  }
  /**
   * @brief マシン回収まで待つ関数
   */
  bool waitForPickup(const int wait_ms = 2000) {
    hw->led->set(0xf);
    for (int ms = 0; ms < wait_ms; ms++) {
      vTaskDelay(pdMS_TO_TICKS(1));
      if (std::abs(hw->imu->get_gyro().x) > thr_gyro_pickup) {
        hw->bz->play(hardware::Buzzer::CANCEL);
        hw->led->set(0x0);
        return true;
      }
    }
    hw->led->set(0x0);
    return false;
  }
  /**
   * @brief 静止状態になるまで待機する関数
   *
   * @return true 静止状態になった
   * @return false キャンセルされた
   */
  bool waitForFix() {
    int fix_count = 0;
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1));
      /* FIX */
      if (std::abs(hw->imu->get_gyro().x) < thr_fix_gyro &&
          std::abs(hw->imu->get_gyro().y) < thr_fix_gyro &&
          std::abs(hw->imu->get_gyro().z) < thr_fix_gyro) {
        if (fix_count++ > wait_fix_ms) {
          hw->bz->play(hardware::Buzzer::CONFIRM);
          return true;
        }
      } else {
        fix_count = 0;
      }
      /* CANCEL */
      if (hw->btn->pressed) {
        hw->btn->flags = 0;
        hw->bz->play(hardware::Buzzer::CANCEL);
        return false;
      }
      if (std::abs(hw->imu->get_accel().x) > thr_accel) {
        hw->bz->play(hardware::Buzzer::CANCEL);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        return false;
      }
    }
  }
  /**
   * @brief Sample the Battery Voltage
   * @return float voltage [V]
   */
  static float getBatteryVoltage() {
    // return 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
    return 2 * peripheral::ADC::read_milli_voltage(BAT_VOL_ADC1_CHANNEL, 10) /
           1e3f;
  }
  /**
   * @brief バッテリー電圧をLEDで表示
   * @param voltage [V]
   */
  void batteryLedIndicate(const float voltage) {
    hw->led->set(0);
    if (voltage < 4.0f)
      hw->led->set(0x01);
    else if (voltage < 4.1f)
      hw->led->set(0x03);
    else if (voltage < 4.2f)
      hw->led->set(0x07);
    else
      hw->led->set(0x0F);
  }
};
