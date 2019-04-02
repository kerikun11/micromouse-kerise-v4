#pragma once

#include "global.h"

#include <Arduino.h>
#include <SPIFFS.h>
#include <cmath>
#include <iostream>
#include <vector>

#define WALL_DETECTOR_TASK_PRIORITY 4
#define WALL_DETECTOR_STACK_SIZE 4096

#define WALL_DETECTOR_BACKUP_PATH "/WallDetector.bin"

#define WALL_DETECTOR_THRESHOLD_FRONT 120
#define WALL_DETECTOR_THRESHOLD_SIDE -20

class WallDetector {
public:
  static constexpr float Ts = 0.001f;
  union WallValue {
    // 意味をもったメンバ
    struct {
      float side[2];
      float front[2];
    };
    // シリアライズされたメンバ
    struct {
      float value[4];
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
  bool begin() {
    calibrationStartSemaphore = xSemaphoreCreateBinary();
    calibrationEndSemaphore = xSemaphoreCreateBinary();
    calibrationFrontStartSemaphore = xSemaphoreCreateBinary();
    calibrationFrontEndSemaphore = xSemaphoreCreateBinary();
    xTaskCreate([](void *obj) { static_cast<WallDetector *>(obj)->task(); },
                "WallDetector", WALL_DETECTOR_STACK_SIZE, this,
                WALL_DETECTOR_TASK_PRIORITY, NULL);
    if (!restore())
      return false;
    return true;
  }
  bool backup() {
    File file = SPIFFS.open(WALL_DETECTOR_BACKUP_PATH, FILE_WRITE);
    if (!file) {
      log_e("Can't open file!");
      return false;
    }
    file.write((const uint8_t *)(&(wall_ref)), sizeof(WallDetector::WallValue));
    return true;
  }
  bool restore() {
    File file = SPIFFS.open(WALL_DETECTOR_BACKUP_PATH, FILE_READ);
    if (!file) {
      log_e("Can't open file!");
      return false;
    }
    if (file.available() == sizeof(WallDetector::WallValue)) {
      uint8_t data[sizeof(WallDetector::WallValue)];
      file.read(data, sizeof(WallDetector::WallValue));
      memcpy((uint8_t *)(&(wall_ref)), data, sizeof(WallDetector::WallValue));
    } else {
      log_e("File size is invalid!");
      return false;
    }
    log_i("WallDetector Restore:\t%.1f\t%.1f\t%.1f\t%.1f", wall_ref.side[0],
          wall_ref.front[0], wall_ref.front[1], wall_ref.side[1]);
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
    log_i("Wall: %5.1f %5.1f %5.1f %5.1f [ %c %c %c ]", distance.side[0],
          distance.front[0], distance.front[1], distance.side[1],
          wall[0] ? 'X' : '.', wall[2] ? 'X' : '.', wall[1] ? 'X' : '.');
  }
  void printDiff() {
    std::cout << diff.side[0] << "," << diff.side[1] << std::endl;
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
  bool wall[3];

private:
  SemaphoreHandle_t calibrationStartSemaphore;
  SemaphoreHandle_t calibrationEndSemaphore;
  SemaphoreHandle_t calibrationFrontStartSemaphore;
  SemaphoreHandle_t calibrationFrontEndSemaphore;
  WallValue wall_ref;
  static const int ave_num = 32;
  Accumulator<WallValue, ave_num> buffer;

  float ref2dist(const int16_t value) {
    return 12.9035f * std::log(value) - 86.7561f;
  }
  void calibration_side() {
    tof.disable();
    float sum[2] = {0.0f, 0.0f};
    const int ave_count = 500;
    for (int j = 0; j < ave_count; j++) {
      for (int i = 0; i < 2; i++)
        sum[i] += ref2dist(ref.side(i));
      delay(1);
    }
    for (int i = 0; i < 2; i++)
      wall_ref.side[i] = sum[i] / ave_count;
    printf("Wall Calibration Side: %6.1f%6.1f\n", wall_ref.side[0],
           wall_ref.side[1]);
    tof.enable();
  }
  void calibration_front() {
    tof.disable();
    float sum[2] = {0.0f, 0.0f};
    const int ave_count = 500;
    for (int j = 0; j < ave_count; j++) {
      for (int i = 0; i < 2; i++)
        sum[i] += ref2dist(ref.front(i));
      delay(1);
    }
    for (int i = 0; i < 2; i++)
      wall_ref.front[i] = sum[i] / ave_count;
    printf("Wall Calibration Front: %6.1f%6.1f\n", wall_ref.front[0],
           wall_ref.front[1]);
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
    if (tof.getDistance() < WALL_DETECTOR_THRESHOLD_FRONT * 0.95f)
      wall[2] = true;
    else if (tof.getDistance() > WALL_DETECTOR_THRESHOLD_FRONT * 1.05f)
      wall[2] = false;
    if (tof.passedTimeMs() > 50)
      wall[2] = false;

    // 横壁の更新
    for (int i = 0; i < 2; i++) {
      const float value = distance.side[i];
      if (value > WALL_DETECTOR_THRESHOLD_SIDE * 0.95f)
        wall[i] = true;
      else if (value < WALL_DETECTOR_THRESHOLD_SIDE * 1.05f)
        wall[i] = false;
    }

    // 変化量の更新
    // diff = (buffer[0] - buffer[ave_num - 1]) / Ts / (ave_num - 1);
    WallValue tmp = {0};
    for (int i = 0; i < ave_num / 2; ++i)
      tmp += buffer[i];
    for (int i = 0; i < ave_num / 2; ++i)
      tmp -= buffer[ave_num / 2 + i];
    diff = tmp / (ave_num / 2) / Ts / (ave_num - 1);
  }
  void task() {
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
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
