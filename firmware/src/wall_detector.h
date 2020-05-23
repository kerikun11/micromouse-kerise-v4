#pragma once

#include "global.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#define WALL_DETECTOR_TASK_PRIORITY 4
#define WALL_DETECTOR_STACK_SIZE 4096

#define WALL_DETECTOR_BACKUP_PATH "/spiffs/WallDetector.bin"

#define WALL_DETECTOR_THRESHOLD_FRONT 135
#define WALL_DETECTOR_THRESHOLD_SIDE -25

class WallDetector {
public:
  static constexpr float Ts = 1e-3f;
  union WallValue {
    // 意味をもったメンバ
    struct {
      float side[2];
      float front[2];
    };
    // シリアライズされたメンバ
    struct {
      float value[4] = {0};
    };
    const WallValue &operator=(const WallValue &wv) {
      for (int i = 0; i < 4; ++i)
        value[i] = wv.value[i];
      return *this;
    }
    const WallValue &operator+=(const WallValue &wv) {
      for (int i = 0; i < 4; ++i)
        value[i] += wv.value[i];
      return *this;
    }
    const WallValue &operator-=(const WallValue &wv) {
      for (int i = 0; i < 4; ++i)
        value[i] -= wv.value[i];
      return *this;
    }
    const WallValue operator-(const WallValue &wv) const {
      WallValue ret;
      for (int i = 0; i < 4; ++i)
        ret.value[i] = value[i] - wv.value[i];
      return ret;
    }
    const WallValue operator/(const auto div) const {
      WallValue ret;
      for (int i = 0; i < 4; ++i)
        ret.value[i] = value[i] / div;
      return ret;
    }
  };

public:
  WallDetector() {}
  bool init() {
    if (!restore())
      return false;
    return true;
  }
  void update() {
    // データの更新
    for (int i = 0; i < 2; i++) {
      distance.side[i] = ref2dist(ref.side(i)) - wall_ref.side[i];
      distance.front[i] = ref2dist(ref.front(i)) - wall_ref.front[i];
    }
    buffer.push(distance);

    // 前壁の更新
    if (tof.getDistance() < WALL_DETECTOR_THRESHOLD_FRONT * 0.95f)
      is_wall[2] = true;
    else if (tof.getDistance() > WALL_DETECTOR_THRESHOLD_FRONT * 1.05f)
      is_wall[2] = false;
    if (tof.passedTimeMs() > 50)
      is_wall[2] = false;

    // 横壁の更新
    for (int i = 0; i < 2; i++) {
      const float value = distance.side[i];
      if (value > WALL_DETECTOR_THRESHOLD_SIDE * 0.95f)
        is_wall[i] = true;
      else if (value < WALL_DETECTOR_THRESHOLD_SIDE * 1.05f)
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
  bool backup() {
    std::ofstream of(WALL_DETECTOR_BACKUP_PATH, std::ios::binary);
    if (of.fail()) {
      log_e("Can't open file!");
      return false;
    }
    of.write((const char *)(&(wall_ref)), sizeof(WallDetector::WallValue));
    return true;
  }
  bool restore() {
    std::ifstream f(WALL_DETECTOR_BACKUP_PATH, std::ios::binary);
    if (f.fail()) {
      log_e("Can't open file!");
      return false;
    }
    if (f.eof()) {
      log_e("File size is invalid!");
      return false;
    }
    f.read((char *)(&wall_ref), sizeof(WallDetector::WallValue));
    logi << "Wall Reference Restore:"
         << "\t" << wall_ref.side[0] << "\t" << wall_ref.front[0] //
         << "\t" << wall_ref.front[1] << "\t" << wall_ref.side[1] //
         << std::endl;
    return true;
  }
  void calibration_side() {
    tof.disable();
    float sum[2] = {0.0f, 0.0f};
    const int ave_count = 500;
    for (int j = 0; j < ave_count; j++) {
      for (int i = 0; i < 2; i++)
        sum[i] += ref2dist(ref.side(i));
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 2; i++)
      wall_ref.side[i] = sum[i] / ave_count;
    logi << "Wall Calibration Side: " << wall_ref.side[0] << "\t"
         << wall_ref.side[1] << std::endl;
    tof.enable();
  }
  void calibration_front() {
    tof.disable();
    float sum[2] = {0.0f, 0.0f};
    const int ave_count = 500;
    for (int j = 0; j < ave_count; j++) {
      for (int i = 0; i < 2; i++)
        sum[i] += ref2dist(ref.front(i));
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 2; i++)
      wall_ref.front[i] = sum[i] / ave_count;
    logi << "Wall Calibration Front: " << wall_ref.front[0] << "\t"
         << wall_ref.front[1] << std::endl;
    tof.enable();
  }
  void print() {
    logi                                                           //
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
    for (int i = 0; i < 4; ++i)
      std::cout << "," << diff.value[i];
    std::cout << std::endl;
  }
  WallValue distance;
  WallValue diff;
  std::array<bool, 3> is_wall;

private:
  WallValue wall_ref;
  static const int ave_num = 32;
  Accumulator<WallValue, ave_num> buffer;

  float ref2dist(const int16_t value) const {
    return 12.9035f * std::log(float(value)) - 86.7561f;
  }
};
