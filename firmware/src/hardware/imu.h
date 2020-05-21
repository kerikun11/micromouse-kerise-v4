#pragma once

#include <icm20602.h>

#include "app_log.h"
#include "config/model.h" //< for KERISE_SELECT

class IMU {
public:
  static constexpr float Ts = 1e-3f;

#if KERISE_SELECT == 4 || KERISE_SELECT == 3
  static constexpr float IMU_ROTATION_RADIOUS = 10.0f;
  static constexpr int IMU_NUM = 2;
#elif KERISE_SELECT == 5
  static constexpr int IMU_NUM = 1;
#endif

public:
  IMU() {}
  bool init(spi_host_device_t spi_host, std::array<int8_t, IMU_NUM> pins_cs) {
    for (int i = 0; i < IMU_NUM; ++i)
      if (!icm[i].init(spi_host, pins_cs[i]))
        return log_e("IMU %d begin failed :(", i), false;
    return true;
  }
  void print() {
    logd << "Gyro:"
         << "\t" << gyro.x //
         << "\t" << gyro.y //
         << "\t" << gyro.z //
         << std::endl;
    logd << "Accel:"
         << "\t" << accel.x //
         << "\t" << accel.y //
         << "\t" << accel.z //
         << std::endl;
    logd << "Angle:\t" << angle << "\t" << angular_accel << std::endl;
  }
  void calibration() {
    for (size_t i = 0; i < IMU_NUM; i++)
      icm[i].calibration();
  }
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

public:
  MotionParameter gyro, accel;
  float angle, angular_accel;

private:
  ICM20602 icm[IMU_NUM];
};
