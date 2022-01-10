/**
 * @file wall_detector.h
 * @brief Wall Detector
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include "hardware/hardware.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

class WallDetector {
public:
  static constexpr float Ts = 1e-3f;
  static constexpr int wall_threshold_front = 135;
  static constexpr int wall_threshold_side = -25;
  static constexpr auto WALL_DETECTOR_BACKUP_PATH = "/spiffs/WallDetector.bin";

  union WallValue {
    // 意味をもったメンバ
    struct {
      float side[2];
      float front[2];
    };
    // シリアライズされたメンバ
    float value[4] = {0, 0, 0, 0};

    WallValue &operator+=(const WallValue &wv) {
      for (int i = 0; i < 4; ++i)
        value[i] += wv.value[i];
      return *this;
    }
    WallValue &operator-=(const WallValue &wv) {
      for (int i = 0; i < 4; ++i)
        value[i] -= wv.value[i];
      return *this;
    }
    WallValue operator-(const WallValue &wv) const {
      WallValue ret;
      for (int i = 0; i < 4; ++i)
        ret.value[i] = value[i] - wv.value[i];
      return ret;
    }
    WallValue operator/(const float div) const {
      WallValue ret;
      for (int i = 0; i < 4; ++i)
        ret.value[i] = value[i] / div;
      return ret;
    }
  };

public:
  WallValue distance;
  WallValue diff;
  std::array<bool, 3> is_wall;

private:
  hardware::Hardware *hw;

public:
  WallDetector(hardware::Hardware *hw) : hw(hw) {}
  bool init() {
    if (!restore())
      return false;
    xTaskCreatePinnedToCore(
        [](void *arg) { static_cast<decltype(this)>(arg)->task(); },
        "WallDetector", 4096, this, 3, NULL, PRO_CPU_NUM);
    return true;
  }
  bool backup() {
    std::ofstream of(WALL_DETECTOR_BACKUP_PATH, std::ios::binary);
    if (of.fail()) {
      app_loge << "Can't open file. " << WALL_DETECTOR_BACKUP_PATH << std::endl;
      return false;
    }
    of.write((const char *)(&(wall_ref)), sizeof(WallDetector::WallValue));
    return true;
  }
  bool restore() {
    std::ifstream f(WALL_DETECTOR_BACKUP_PATH, std::ios::binary);
    if (f.fail()) {
      app_loge << "Can't open file. " << WALL_DETECTOR_BACKUP_PATH << std::endl;
      return false;
    }
    if (f.eof()) {
      app_loge << "invalid file size. " << WALL_DETECTOR_BACKUP_PATH
               << std::endl;
      return false;
    }
    f.read((char *)(&wall_ref), sizeof(WallDetector::WallValue));
    app_logi << "Wall Reference Restore:"
             << "\t" << wall_ref.side[0] << "\t" << wall_ref.front[0] //
             << "\t" << wall_ref.front[1] << "\t" << wall_ref.side[1] //
             << std::endl;
    return true;
  }
  void calibration_side() {
    hw->tof->disable();
    float sum[2] = {0.0f, 0.0f};
    const int ave_count = 500;
    for (int j = 0; j < ave_count; j++) {
      for (int i = 0; i < 2; i++)
        sum[i] += ref2dist(hw->ref->side(i));
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 2; i++)
      wall_ref.side[i] = sum[i] / ave_count;
    app_logi << "Wall Calibration Side: " << wall_ref.side[0] << "\t"
             << wall_ref.side[1] << std::endl;
    hw->tof->enable();
  }
  void calibration_front() {
    hw->tof->disable();
    float sum[2] = {0.0f, 0.0f};
    const int ave_count = 500;
    for (int j = 0; j < ave_count; j++) {
      for (int i = 0; i < 2; i++)
        sum[i] += ref2dist(hw->ref->front(i));
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 2; i++)
      wall_ref.front[i] = sum[i] / ave_count;
    app_logi << "Wall Calibration Front: " << wall_ref.front[0] << "\t"
             << wall_ref.front[1] << std::endl;
    hw->tof->enable();
  }
  void print() {
    app_logi                                                       //
        << std::setw(10) << std::setfill(' ') << distance.side[0]  //
        << std::setw(10) << std::setfill(' ') << distance.front[0] //
        << std::setw(10) << std::setfill(' ') << distance.front[1] //
        << std::setw(10) << std::setfill(' ') << distance.side[1]  //
        << "\t" << (is_wall[0] ? "X" : "_")                        //
        << " " << (is_wall[2] ? "X" : "_")                         //
        << " " << (is_wall[1] ? "X" : "_")                         //
        << std::endl;
  }
  void csv() {
    std::cout << "0";
    for (int i = 0; i < 4; ++i)
      std::cout << "," << distance.value[i];
    std::cout << std::endl;
  }

private:
  WallValue wall_ref;
  static const int ave_num = 32;
  ctrl::Accumulator<WallValue, ave_num> buffer;

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();
    }
  }
  void update() {
    // データの更新
    for (int i = 0; i < 2; i++) {
      distance.side[i] = ref2dist(hw->ref->side(i)) - wall_ref.side[i];
      distance.front[i] = ref2dist(hw->ref->front(i)) - wall_ref.front[i];
    }
    buffer.push(distance);

    // 前壁の更新
    // int front_mm = hw->tof->getLog().average(2);
    int front_mm = hw->tof->getDistance();
    if (front_mm < wall_threshold_front * 0.95f)
      is_wall[2] = true;
    else if (front_mm > wall_threshold_front * 1.05f)
      is_wall[2] = false;
    if (hw->tof->passedTimeMs() > 50)
      is_wall[2] = false;

    // 横壁の更新
    for (int i = 0; i < 2; i++) {
      const float value = distance.side[i];
      if (value > wall_threshold_side * 0.95f)
        is_wall[i] = true;
      else if (value < wall_threshold_side * 1.05f)
        is_wall[i] = false;
    }

    // 変化量の更新
    // diff = (buffer[0] - buffer[ave_num - 1]) / Ts / (ave_num - 1);
    WallValue tmp;
    for (int i = 0; i < ave_num / 2; ++i)
      tmp += buffer[i];
    for (int i = 0; i < ave_num / 2; ++i)
      tmp -= buffer[ave_num / 2 + i];
    diff = tmp / (ave_num / 2) / Ts / (ave_num - 1);
  }

  float ref2dist(const int16_t value) const {
    return 12.9035f * std::log(float(value)) - 86.7561f;
  }
};
