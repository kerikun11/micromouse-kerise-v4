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
  static constexpr float Ts = float(1e-3);
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
    calibrationStartSemaphore = xSemaphoreCreateBinary();
    calibrationEndSemaphore = xSemaphoreCreateBinary();
    calibrationFrontStartSemaphore = xSemaphoreCreateBinary();
    calibrationFrontEndSemaphore = xSemaphoreCreateBinary();
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "WallDetector", WALL_DETECTOR_STACK_SIZE, this,
                WALL_DETECTOR_TASK_PRIORITY, NULL);
    if (!restore())
      return false;
    return true;
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
  void calibrationSide() {
    xSemaphoreTake(calibrationEndSemaphore,
                   0); //< 前のキャリブレーションの終了を待つ
    xSemaphoreGive(calibrationStartSemaphore);
    xSemaphoreTake(calibrationEndSemaphore, portMAX_DELAY);
  }
  void calibrationFront() {
    xSemaphoreTake(calibrationFrontEndSemaphore,
                   0); //< 前のキャリブレーションの終了を待つ
    xSemaphoreGive(calibrationFrontStartSemaphore);
    xSemaphoreTake(calibrationFrontEndSemaphore, portMAX_DELAY);
  }
  void print() {
    // log_i("Wall: %5.1f %5.1f %5.1f %5.1f [ %c %c %c ]", distance.side[0],
    //       distance.front[0], distance.front[1], distance.side[1],
    //       is_wall[0] ? 'X' : '.', is_wall[2] ? 'X' : '.',
    //       is_wall[1] ? 'X' : '.');
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
  SemaphoreHandle_t calibrationStartSemaphore;
  SemaphoreHandle_t calibrationEndSemaphore;
  SemaphoreHandle_t calibrationFrontStartSemaphore;
  SemaphoreHandle_t calibrationFrontEndSemaphore;
  WallValue wall_ref;
  static const int ave_num = 32;
  Accumulator<WallValue, ave_num> buffer;

  float ref2dist(const int16_t value) {
    return float(12.9035) * std::log(float(value)) - float(86.7561);
  }
  void calibration_side() {
    tof.disable();
    float sum[2] = {float(0), float(0)};
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
    float sum[2] = {float(0), float(0)};
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
  void update() {
    // データの更新
    for (int i = 0; i < 2; i++) {
      distance.side[i] = ref2dist(ref.side(i)) - wall_ref.side[i];
      distance.front[i] = ref2dist(ref.front(i)) - wall_ref.front[i];
    }
    buffer.push(distance);

    // 前壁の更新
    if (tof.getDistance() < WALL_DETECTOR_THRESHOLD_FRONT * float(0.95))
      is_wall[2] = true;
    else if (tof.getDistance() > WALL_DETECTOR_THRESHOLD_FRONT * float(1.05))
      is_wall[2] = false;
    if (tof.passedTimeMs() > 50)
      is_wall[2] = false;

    // 横壁の更新
    for (int i = 0; i < 2; i++) {
      const float value = distance.side[i];
      if (value > WALL_DETECTOR_THRESHOLD_SIDE * float(0.95))
        is_wall[i] = true;
      else if (value < WALL_DETECTOR_THRESHOLD_SIDE * float(1.05))
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
  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      // データの更新
      update();

      // キャリブレーションがリクエストされていたらする
      if (xSemaphoreTake(calibrationStartSemaphore, 0) == pdTRUE) {
        calibration_side();
        xSemaphoreGive(calibrationEndSemaphore);
      }
      if (xSemaphoreTake(calibrationFrontStartSemaphore, 0) == pdTRUE) {
        calibration_front();
        xSemaphoreGive(calibrationFrontEndSemaphore);
      }
    }
  }
};
