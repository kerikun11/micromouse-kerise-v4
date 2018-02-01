#pragma once

#include <Arduino.h>
#include "icm20602.h"

#define IMU_UPDATE_PERIOD_US  1000
#define IMU_STACK_SIZE        2048
#define IMU_TASK_PRIORITY     5
#define IMU_ROTATION_RADIOUS  10.0f

class IMU {
  public:
    IMU() {
      calibration_start_semaphore = xSemaphoreCreateBinary();
      calibration_end_semaphore = xSemaphoreCreateBinary();
    }
    bool begin(spi_host_device_t spi_host,
               std::array<int8_t, 2> pins_cs,
               bool spi_bus_initializing,
               int8_t pin_sclk, int8_t pin_miso, int8_t pin_mosi,
               int dma_chain = 0) {
      if (spi_bus_initializing) {
        // ESP-IDF SPI bus initialization
        spi_bus_config_t bus_cfg = {0};
        bus_cfg.mosi_io_num = pin_mosi; ///< GPIO pin for Master Out Slave In (=spi_d) signal, or -1 if not used.
        bus_cfg.miso_io_num = pin_miso; ///< GPIO pin for Master In Slave Out (=spi_q) signal, or -1 if not used.
        bus_cfg.sclk_io_num = pin_sclk; ///< GPIO pin for Spi CLocK signal, or -1 if not used.
        bus_cfg.quadwp_io_num = -1;     ///< GPIO pin for WP (Write Protect) signal which is used as D2 in 4-bit communication modes, or -1 if not used.
        bus_cfg.quadhd_io_num = -1;     ///< GPIO pin for HD (HolD) signal which is used as D3 in 4-bit communication modes, or -1 if not used.
        bus_cfg.max_transfer_sz = 0;    ///< Maximum transfer size, in bytes. Defaults to 4094 if 0.
        ESP_ERROR_CHECK(spi_bus_initialize(spi_host, &bus_cfg, dma_chain));
      }
      if (!icm[0].begin(spi_host, pins_cs[0]) || !icm[1].begin(spi_host, pins_cs[1])) {
        log_e("IMU begin failed :(");
        return false;
      }
      xTaskCreate([](void* obj) {
        static_cast<IMU*>(obj)->task();
      }, "IMU", IMU_STACK_SIZE, this, IMU_TASK_PRIORITY, NULL);
      return true;
    }
    void print() {
      log_d("Rotation angle:\t%f\taccel:\t%f", angle, angular_accel);
      log_d("Gyro\tx:\t%fy:\t%f\tz:\t%f", gyro.x, gyro.y, gyro.z);
      log_d("Accel\tx:\t%fy:\t%f\tz:\t%f", accel.x, accel.y, accel.z);
    }
    void calibration(bool waitForEnd = true) {
      xSemaphoreTake(calibration_end_semaphore, 0);
      xSemaphoreGive(calibration_start_semaphore);
      if (waitForEnd) calibrationWait();
    }
    void calibrationWait() {
      xSemaphoreTake(calibration_end_semaphore, portMAX_DELAY);
    }

  public:
    MotionParameter gyro, accel;
    float angle, angular_accel;
  private:
    xTaskHandle task_handle;
    SemaphoreHandle_t calibration_start_semaphore;
    SemaphoreHandle_t calibration_end_semaphore;
    ICM20602 icm[2];

    void update() {
      icm[0].update();
      icm[1].update();
      gyro.x = (-icm[0].gyro.x + icm[1].gyro.x) / 2;
      gyro.y = (-icm[0].gyro.y + icm[1].gyro.y) / 2;
      gyro.z = (icm[0].gyro.z + icm[1].gyro.z) / 2;
      accel.x = (-icm[0].accel.x + icm[1].accel.x) / 2;
      accel.y = (-icm[0].accel.y + icm[1].accel.y) / 2;
      accel.z = (icm[0].accel.z + icm[1].accel.z) / 2;
      angle += gyro.z * IMU_UPDATE_PERIOD_US / 1000000;
      angular_accel = (icm[0].accel.y + icm[1].accel.y) / 2 / IMU_ROTATION_RADIOUS;
    }
    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        update();

        if (xSemaphoreTake(calibration_start_semaphore, 0) == pdTRUE) {
          icm[0].calibration();
          icm[1].calibration();
          xSemaphoreGive(calibration_end_semaphore);
        }
      }
    }
};

