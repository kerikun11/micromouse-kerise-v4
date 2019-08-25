#pragma once

#include <array>
#include <driver/spi_master.h>
#include <esp_err.h>

#include "spi.h"

#include "config/model.h"

struct MotionParameter {
  float x, y, z;
  MotionParameter(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
  const MotionParameter operator+(const MotionParameter &obj) const {
    return MotionParameter(x + obj.x, y + obj.y, z + obj.z);
  }
  const MotionParameter operator*(const float mul) const {
    return MotionParameter(x * mul, y * mul, z * mul);
  }
  const MotionParameter operator/(const float div) const {
    return MotionParameter(x / div, y / div, z / div);
  }
  const MotionParameter &operator=(const MotionParameter &obj) {
    x = obj.x;
    y = obj.y;
    z = obj.z;
    return *this;
  }
  const MotionParameter &operator+=(const MotionParameter &obj) {
    x += obj.x;
    y += obj.y;
    z += obj.z;
    return *this;
  }
};

class ICM20602 {
public:
  static constexpr float ACCEL_G = 9806.65f; //< [mm/s/s]
#ifndef M_PI
  static constexpr float M_PI = 3.1415926535897932384626433832795f;
#endif

private:
  static constexpr float ICM20602_ACCEL_FACTOR = 2048.0f;
  static constexpr float ICM20602_GYRO_FACTOR = 16.4f;

public:
  ICM20602() {}
  MotionParameter accel, gyro;

public:
  bool begin(spi_host_device_t spi_host, int8_t pin_cs) {
    // ESP-IDF SPI device initialization
    static spi_device_interface_config_t dev_cfg;
    dev_cfg.command_bits = 0;
    dev_cfg.address_bits = 8;
    dev_cfg.dummy_bits = 0;
    dev_cfg.mode = 3;
    dev_cfg.duty_cycle_pos = 0;
    dev_cfg.cs_ena_pretrans = 0;
    dev_cfg.cs_ena_posttrans = 0;
    dev_cfg.clock_speed_hz = 20000000;
    dev_cfg.spics_io_num = pin_cs;
    dev_cfg.flags = 0;
    dev_cfg.queue_size = 2;
    dev_cfg.pre_cb = NULL;
    dev_cfg.post_cb = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &spi_handle));
    return reset();
  }
  void device_reset() { writeReg(107, 0x81); /**< PowerManagement 1 */ }
  void device_config() {
    writeReg(26, 0x00); /**< Config; DLPF=250[Hz] */
    writeReg(27, 0x18); /**< Gyro Config; FS=2000[dps], FCHOICE=8[kHz] */
    writeReg(28, 0x18); /**< Accel Config; FS=16[g] */
    writeReg(29, 0x00); /**< Accel Config 2; F_CHOICE=1[kHz], DLPF=218.1[Hz] */
    // writeReg(17, 0xc9);  /**< ??? for Accel */
    writeReg(107, 0x01); /**< PowerManagement 1 */
  }
  bool reset() {
    device_reset();
    delay(100);
    device_config();
    return whoami();
  }
  bool whoami() {
    uint8_t v = readReg(117); /**< Who am I */
    if (v != 0x12) {
      log_e("whoami failed:( 0x%X", v);
      return false;
    }
    return true;
  }
  void update() {
    union {
      int16_t i;
      struct {
        uint8_t l : 8;
        uint8_t h : 8;
      };
    } bond;
    uint8_t rx[14];
    readReg(0x3b, rx, 14);
    bond.h = rx[0];
    bond.l = rx[1];
    accel.x = bond.i / ICM20602_ACCEL_FACTOR * ACCEL_G - accel_offset.x;
    bond.h = rx[2];
    bond.l = rx[3];
    accel.y = bond.i / ICM20602_ACCEL_FACTOR * ACCEL_G - accel_offset.y;
    bond.h = rx[4];
    bond.l = rx[5];
    accel.z = bond.i / ICM20602_ACCEL_FACTOR * ACCEL_G - accel_offset.z;

    bond.h = rx[8];
    bond.l = rx[9];
    gyro.x = bond.i / ICM20602_GYRO_FACTOR * M_PI / 180 - gyro_offset.x;
    bond.h = rx[10];
    bond.l = rx[11];
    gyro.y = bond.i / ICM20602_GYRO_FACTOR * M_PI / 180 - gyro_offset.y;
    bond.h = rx[12];
    bond.l = rx[13];
    gyro.z = bond.i / ICM20602_GYRO_FACTOR * M_PI / 180 - gyro_offset.z;
  }
  void calibration() {
    const int ave_count = 100;
    for (int j = 0; j < 2; j++) {
      portTickType xLastWakeTime = xTaskGetTickCount();
      MotionParameter accel_sum, gyro_sum;
      for (int i = 0; i < ave_count; i++) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        update();
        accel_sum += accel;
        gyro_sum += gyro;
      }
      accel_offset += accel_sum / ave_count;
      gyro_offset += gyro_sum / ave_count;
    }
  }

private:
  spi_device_handle_t spi_handle;
  MotionParameter accel_offset, gyro_offset;

  void writeReg(uint8_t reg, uint8_t data) {
    static spi_transaction_t tx;
    tx.flags |= SPI_TRANS_USE_TXDATA;
    tx.addr = reg;
    tx.tx_data[0] = data;
    tx.length = 8;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
  }
  uint8_t readReg(const uint8_t reg) {
    static spi_transaction_t tx;
    tx.flags |= SPI_TRANS_USE_RXDATA;
    tx.addr = 0x80 | reg;
    tx.length = 8;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
    return tx.rx_data[0];
  }
  void readReg(const uint8_t reg, uint8_t *rx_buffer, size_t length) {
    static spi_transaction_t tx;
    tx.addr = 0x80 | reg;
    tx.tx_buffer = NULL;
    tx.rx_buffer = rx_buffer;
    tx.length = 8 * length;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
  }
};

#if KERISE_SELECT == 4

#define IMU_STACK_SIZE 2048
#define IMU_TASK_PRIORITY 5

class IMU {
public:
  static constexpr float Ts = 0.001f;
  static constexpr float IMU_ROTATION_RADIOUS = 10.0f;

public:
  IMU() {
    sampling_end_semaphore = xSemaphoreCreateBinary();
    calibration_start_semaphore = xSemaphoreCreateBinary();
    calibration_end_semaphore = xSemaphoreCreateBinary();
  }
  bool begin(spi_host_device_t spi_host, std::array<int8_t, 2> pins_cs) {
    if (!icm[0].begin(spi_host, pins_cs[0])) {
      log_e("IMU 0 begin failed :(");
      return false;
    }
    if (!icm[1].begin(spi_host, pins_cs[1])) {
      log_e("IMU 1 begin failed :(");
      return false;
    }
    xTaskCreate([](void *obj) { static_cast<IMU *>(obj)->task(); }, "IMU",
                IMU_STACK_SIZE, this, IMU_TASK_PRIORITY, NULL);
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
  void samplingSemaphoreTake(portTickType xBlockTime = portMAX_DELAY) {
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
  ICM20602 icm[2];

  void update() {
    icm[0].update();
    icm[1].update();

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

#if 1
    gyro.x = (-icm[0].gyro.x + icm[1].gyro.x) / 2;
    gyro.y = (-icm[0].gyro.y + icm[1].gyro.y) / 2;
    gyro.z = (icm[0].gyro.z + icm[1].gyro.z) / 2;
    accel.x = (-icm[0].accel.x + icm[1].accel.x) / 2;
    accel.y = (-icm[0].accel.y + icm[1].accel.y) / 2;
    accel.z = (icm[0].accel.z + icm[1].accel.z) / 2;
#endif

    angle += gyro.z * Ts;
    angular_accel =
        (icm[0].accel.y + icm[1].accel.y) / 2 / IMU_ROTATION_RADIOUS;
  }
  void task() {
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
      xLastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); //< 同期
      update();                               //< データの更新
      xSemaphoreGive(sampling_end_semaphore); //< サンプリング終了を知らせる

      // キャリブレーションが要求されていたら行う
      if (xSemaphoreTake(calibration_start_semaphore, 0) == pdTRUE) {
        icm[0].calibration();
        icm[1].calibration();
        // キャリブレーション終了を知らせるセマフォ
        xSemaphoreGive(calibration_end_semaphore);
      }
    }
  }
};

#elif KERISE_SELECT == 5

#define IMU_STACK_SIZE 2048
#define IMU_TASK_PRIORITY 5

class IMU {
public:
  static constexpr float Ts = 0.001f;

public:
  IMU() {
    sampling_end_semaphore = xSemaphoreCreateBinary();
    calibration_start_semaphore = xSemaphoreCreateBinary();
    calibration_end_semaphore = xSemaphoreCreateBinary();
  }
  bool begin(spi_host_device_t spi_host, gpio_num_t pin_cs) {
    if (!icm.begin(spi_host, pin_cs)) {
      log_e("IMU begin failed :(");
      return false;
    }
    xTaskCreate([](void *obj) { static_cast<IMU *>(obj)->task(); }, "IMU",
                IMU_STACK_SIZE, this, IMU_TASK_PRIORITY, NULL);
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
  void samplingSemaphoreTake(portTickType xBlockTime = portMAX_DELAY) {
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
  ICM20602 icm;

  void update() {
    icm.update();

    gyro.x = icm.gyro.x;
    gyro.y = icm.gyro.y;
    gyro.z = icm.gyro.z;
    accel.x = icm.accel.x;
    accel.y = icm.accel.y;
    accel.z = icm.accel.z;

    angle += gyro.z * Ts;
  }
  void task() {
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
      xLastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); //< 同期
      update();                               //< データの更新
      xSemaphoreGive(sampling_end_semaphore); //< サンプリング終了を知らせる

      // キャリブレーションが要求されていたら行う
      if (xSemaphoreTake(calibration_start_semaphore, 0) == pdTRUE) {
        icm.calibration();
        // キャリブレーション終了を知らせるセマフォ
        xSemaphoreGive(calibration_end_semaphore);
      }
    }
  }
};

#endif
