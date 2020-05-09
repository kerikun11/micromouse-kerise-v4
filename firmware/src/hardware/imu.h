#pragma once

#include <icm20602.h>

#include "config/model.h" //< for KERISE_SELECT

class IMU {
public:
  static constexpr int IMU_STACK_SIZE = 2048;
  static constexpr int IMU_TASK_PRIORITY = 5;
  static constexpr float Ts = 0.001f;

#if KERISE_SELECT == 4 || KERISE_SELECT == 3
  static constexpr float IMU_ROTATION_RADIOUS = 10.0f;
  static constexpr int IMU_NUM = 2;
#elif KERISE_SELECT == 5
  static constexpr int IMU_NUM = 1;
#endif

public:
  IMU() {
    sampling_end_semaphore = xSemaphoreCreateBinary();
    calibration_start_semaphore = xSemaphoreCreateBinary();
    calibration_end_semaphore = xSemaphoreCreateBinary();
  }
  bool begin(spi_host_device_t spi_host, std::array<int8_t, IMU_NUM> pins_cs) {
    for (int i = 0; i < IMU_NUM; ++i)
      if (!icm[i].begin(spi_host, pins_cs[i])) {
        log_e("IMU %d begin failed :(", i);
        return false;
      }
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "IMU", IMU_STACK_SIZE, this, IMU_TASK_PRIORITY, NULL);
    return true;
  }
  void print() {
    log_d("Rotation angle:\t%f\taccel:\t%f", angle, angular_accel);
    log_d("Gyro\tx:\t%f\ty:\t%f\tz:\t%f", gyro.x, gyro.y, gyro.z);
    log_d("Accel\tx:\t%f\ty:\t%f\tz:\t%f", accel.x, accel.y, accel.z);
  }
  void csv() { printf("%.1f,%.1f,%.1f\n", accel.x, accel.y, accel.z); }
  void calibration(bool waitForEnd = true) {
    // 前のフラグが残っていたら回収
    xSemaphoreTake(calibration_end_semaphore, 0);
    xSemaphoreGive(calibration_start_semaphore);
    if (waitForEnd)
      calibrationWait();
  }
  void calibrationWait() {
    xSemaphoreTake(calibration_end_semaphore, portMAX_DELAY);
  }
  void samplingSemaphoreTake(TickType_t xBlockTime = portMAX_DELAY) {
    xSemaphoreTake(sampling_end_semaphore, xBlockTime);
  }

public:
  MotionParameter gyro, accel;
  float angle, angular_accel;

private:
  SemaphoreHandle_t
      sampling_end_semaphore; //< サンプリング終了を知らせるセマフォ
  SemaphoreHandle_t
      calibration_start_semaphore; //< キャリブレーション要求を知らせるセマフォ
  SemaphoreHandle_t
      calibration_end_semaphore; //< キャリブレーション終了を知らせるセマフォ
  ICM20602 icm[IMU_NUM];

  void update() {
#if KERISE_SELECT == 4 || KERISE_SELECT == 3
    for (size_t i = 0; i < IMU_NUM; i++)
      icm[i].update();

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

    angle += gyro.z * Ts;
    angular_accel =
        (icm[0].accel.y + icm[1].accel.y) / 2 / IMU_ROTATION_RADIOUS;

#elif KERISE_SELECT == 5
    for (size_t i = 0; i < IMU_NUM; i++)
      icm[i].update();
    const auto prev_gyro_z = gyro.z;

    gyro.x = icm[0].gyro.x;
    gyro.y = icm[0].gyro.y;
    gyro.z = icm[0].gyro.z;
    accel.x = icm[0].accel.x;
    accel.y = icm[0].accel.y;
    accel.z = icm[0].accel.z;

    angle += gyro.z * Ts;
    angular_accel = (gyro.z - prev_gyro_z) / Ts;
#endif
  }
  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();                               //< データの更新
      xSemaphoreGive(sampling_end_semaphore); //< サンプリング終了を知らせる

      // キャリブレーションが要求されていたら行う
      if (xSemaphoreTake(calibration_start_semaphore, 0) == pdTRUE) {
        for (size_t i = 0; i < IMU_NUM; i++)
          icm[i].calibration();
        // キャリブレーション終了を知らせるセマフォ
        xSemaphoreGive(calibration_end_semaphore);
      }
    }
  }
};
