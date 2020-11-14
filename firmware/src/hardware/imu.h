#pragma once

#include "app_log.h"
#include "config/model.h" //< for KERISE_SELECT

#include <freertospp/semphr.h>
#include <icm20602.h>

#include <atomic>

class IMU {
public:
  static constexpr float Ts = 1e-3f;

#if KERISE_SELECT == 4 || KERISE_SELECT == 3
  static constexpr float IMU_ROTATION_RADIUS = 10.0f;
  static constexpr int IMU_NUM = 2;
#elif KERISE_SELECT == 5
  static constexpr int IMU_NUM = 1;
#endif

public:
  MotionParameter gyro, accel;
  float angle, angular_accel;

public:
  IMU() {}
  bool init(spi_host_device_t spi_host, std::array<int8_t, IMU_NUM> pins_cs) {
    for (int i = 0; i < IMU_NUM; ++i)
      if (!icm[i].init(spi_host, pins_cs[i])) {
        loge << "IMU " << i << " begin failed :(" << std::endl;
        return false;
      }
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "IMU", 4096, this, 6, NULL);
    return true;
  }
  void calibration(const bool wait_for_end = true) {
    calibration_requested = true;
    // calibration will be conducted in background task()
    while (wait_for_end && calibration_requested)
      vTaskDelay(pdMS_TO_TICKS(1));
  }
  void sampling_sync(portTickType xBlockTime = portMAX_DELAY) const {
    sampling_end_semaphore.take(xBlockTime);
  }
  void print() {
    logi << "Gyro:"
         << "\t" << gyro.x //
         << "\t" << gyro.y //
         << "\t" << gyro.z //
         << std::endl;
    logi << "Accel:"
         << "\t" << accel.x //
         << "\t" << accel.y //
         << "\t" << accel.z //
         << std::endl;
    logi << "Angle:\t" << angle << "\t" << angular_accel << std::endl;
  }
  void csv() {
    std::cout << "0," << gyro.x << "," << gyro.y << "," << gyro.z << std::endl;
  }

private:
  ICM20602 icm[IMU_NUM];
  std::atomic_bool calibration_requested{false};
  freertospp::Semaphore sampling_end_semaphore;
  MotionParameter gyro_offset, accel_offset;

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      /* sync */
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      /* fetch */
      update();
      /* give */
      sampling_end_semaphore.give();
      /* calibration */
      if (calibration_requested) {
        task_calibration(xLastWakeTime);
        calibration_requested = false;
      }
    }
  }

  void task_calibration(TickType_t &xLastWakeTime) {
    const int ave_count = 200;
    for (int j = 0; j < 2; j++) {
      MotionParameter accel_sum, gyro_sum;
      for (int i = 0; i < ave_count; i++) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
        update();
        sampling_end_semaphore.give();
        accel_sum += accel;
        gyro_sum += gyro;
      }
      accel_offset += accel_sum / ave_count;
      gyro_offset += gyro_sum / ave_count;
    }
  }
  void update() {
    for (size_t i = 0; i < IMU_NUM; i++)
      icm[i].update();

#if KERISE_SELECT == 4 || KERISE_SELECT == 3
#if 1
    gyro.x = (-icm[0].gyro.x + icm[1].gyro.x) / 2;
    gyro.y = (-icm[0].gyro.y + icm[1].gyro.y) / 2;
    gyro.z = (icm[0].gyro.z + icm[1].gyro.z) / 2;
    accel.x = (-icm[0].accel.x + icm[1].accel.x) / 2;
    accel.y = (-icm[0].accel.y + icm[1].accel.y) / 2;
    accel.z = (icm[0].accel.z + icm[1].accel.z) / 2;
#endif
#if 0
    gyro.x = -icm[0].gyro.x;
    gyro.y = -icm[0].gyro.y;
    gyro.z = icm[0].gyro.z;
    accel.x = -icm[0].accel.x;
    accel.y = -icm[0].accel.y;
    accel.z = icm[0].accel.z;
#endif
#if 0
    gyro.x = icm[1].gyro.x;
    gyro.y = icm[1].gyro.y;
    gyro.z = icm[1].gyro.z;
    accel.x = icm[1].accel.x;
    accel.y = icm[1].accel.y;
    accel.z = icm[1].accel.z;
#endif
#elif KERISE_SELECT == 5
    const auto prev_gyro_z = gyro.z;

    gyro.x = icm[0].gyro.x;
    gyro.y = icm[0].gyro.y;
    gyro.z = icm[0].gyro.z;
    accel.x = icm[0].accel.x;
    accel.y = icm[0].accel.y;
    accel.z = icm[0].accel.z;
#endif

    /* calibration result */
    gyro -= gyro_offset;
    accel -= accel_offset;

#if KERISE_SELECT == 4 || KERISE_SELECT == 3
    angle += gyro.z * Ts;
    angular_accel = (icm[0].accel.y + icm[1].accel.y) / 2 / IMU_ROTATION_RADIUS;
#elif KERISE_SELECT == 5
    angle += gyro.z * Ts;
    angular_accel = (gyro.z - prev_gyro_z) / Ts;
#endif
  }
};
