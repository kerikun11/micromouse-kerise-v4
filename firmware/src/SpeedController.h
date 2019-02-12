#pragma once

#include "global.h"

#include "Accumulator.h"
#include <cmath>

class Position {
public:
  float x, y, theta;

public:
  Position(const float x = 0, const float y = 0, const float theta = 0)
      : x(x), y(y), theta(theta) {}
  Position(const Position &obj) : x(obj.x), y(obj.y), theta(obj.theta) {}
  Position(const float pos[3]) : x(pos[0]), y(pos[1]), theta(pos[2]) {}
  void clear() { x = y = theta = 0; }
  void set(const float x = 0, const float y = 0, const float theta = 0) {
    this->x = x;
    this->y = y;
    this->theta = theta;
  }
  const Position rotate(const float angle) const {
    return Position(x * std::cos(angle) - y * std::sin(angle),
                    x * std::sin(angle) + y * std::cos(angle), theta);
  }
  float getNorm() const { return std::sqrt(x * x + y * y); }
  const Position mirror_x() const { return Position(x, -y, -theta); }
  const Position &operator=(const Position &obj) {
    x = obj.x;
    y = obj.y;
    theta = obj.theta;
    return *this;
  }
  const Position operator+() const { return Position(x, y, theta); }
  const Position operator+(const Position &obj) const {
    return Position(x + obj.x, y + obj.y, theta + obj.theta);
  }
  const Position &operator+=(const Position &obj) {
    x += obj.x;
    y += obj.y;
    theta += obj.theta;
    return *this;
  }
  const Position operator-() const { return Position(-x, -y, -theta); }
  const Position operator-(const Position &obj) const {
    return Position(x - obj.x, y - obj.y, theta - obj.theta);
  }
  const Position &operator-=(const Position &obj) {
    x -= obj.x;
    y -= obj.y;
    theta -= obj.theta;
    return *this;
  }
  void print(const char *name = "Pos") {
    printf("%s: (%.1f,\t%.1f,\t%.3f)\n", name, x, y, theta);
  }
};

#define SPEED_CONTROLLER_TASK_PRIORITY 4
#define SPEED_CONTROLLER_STACK_SIZE 4096

#include <pid_controller.h>

class SpeedController {
public:
  constexpr static const float Ts = 0.001f;
  struct WheelParameter {
  public:
    float tra;      //< tralation [mm]
    float rot;      //< rotation [rad]
    float wheel[2]; //< wheel position [mm], wheel[0]:left, wheel[1]:right
  public:
    WheelParameter() { clear(); }
    void pole2wheel() {
      wheel[0] = tra - MACHINE_ROTATION_RADIUS * rot;
      wheel[1] = tra + MACHINE_ROTATION_RADIUS * rot;
    }
    void wheel2pole() {
      rot = (wheel[1] - wheel[0]) / 2 / MACHINE_ROTATION_RADIUS;
      tra = (wheel[1] + wheel[0]) / 2;
    }
    void clear() {
      tra = 0;
      rot = 0;
      wheel[0] = 0;
      wheel[1] = 0;
    }
  };
  WheelParameter target_v;
  WheelParameter target_a;
  WheelParameter est_v;
  WheelParameter est_a;
  WheelParameter enc_v;
  WheelParameter proportional;
  WheelParameter integral;
  WheelParameter differential;
  WheelParameter pwm_value;
  Position position;
  // Disc2DOFPIDController pidc_tra;
  // Disc2DOFPIDController pidc_rot;
  PIDController pidc_tra;
  PIDController pidc_rot;

public:
  SpeedController() {
    enabled = false;
    reset();
    // Disc2DOFPIDController::Parameter gain_tra(0.897f * 0.001f, 2.07f *
    // 0.001f,
    //                                           0.0665f * 0.001f, 0.0413f,
    //                                           0.265f, 0.004f, 0.001f);
    // Disc2DOFPIDController::Parameter gain_rot(4.21e-5f, 0.0842f, 0.0f, 0.0f,
    //                                           0.0f, 0.0f, 0.001f);
    PIDController::Parameter gain_tra(0.0006f, 0.12f, 0.00003f);
    PIDController::Parameter gain_rot(0.0006f, 0.12f, 0.00003f);
    pidc_tra.setGain(gain_tra);
    pidc_rot.setGain(gain_rot);
    xTaskCreate([](void *obj) { static_cast<SpeedController *>(obj)->task(); },
                "SpeedController", SPEED_CONTROLLER_STACK_SIZE, this,
                SPEED_CONTROLLER_TASK_PRIORITY, NULL);
  }
  void enable(const bool &reset_position = true) {
    reset();
    if (reset_position)
      position.clear();
    enabled = true;
    printf("Speed Controller Enabled\n");
  }
  void disable() {
    enabled = false;
    delay(2);
    mt.free();
    printf("Speed Controller disabled\n");
  }
  void set_target(const float tra, const float rot, const float tra_a = 0,
                  const float rot_a = 0) {
    target_v.tra = tra;
    target_v.rot = rot;
    target_v.pole2wheel();
    target_a.tra = tra_a;
    target_a.rot = rot_a;
    target_a.pole2wheel();
  }

private:
  bool enabled = false;
  static const int acc_num = 8;
  Accumulator<float, acc_num> wheel_position[2];
  Accumulator<float, acc_num> accel;
  Accumulator<float, acc_num> gyro;
  Accumulator<float, acc_num> angular_accel;

  void reset() {
    target_v.clear();
    est_v.clear();
    integral.clear();
    differential.clear();
    for (int i = 0; i < 2; i++)
      wheel_position[i].clear(enc.position(i));
    accel.clear(imu.accel.y);
    gyro.clear(imu.gyro.z);
    angular_accel.clear(imu.angular_accel);
  }
  void task() {
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      // 有効でなければスルー
      if (enabled == false)
        continue;
      // サンプリング終了まで待つ
      enc.samplingSemaphoreTake();
      imu.samplingSemaphoreTake();
      // 最新のデータの追加
      for (int i = 0; i < 2; i++)
        wheel_position[i].push(enc.position(i));
      accel.push(imu.accel.y);
      gyro.push(imu.gyro.z);
      angular_accel.push(imu.angular_accel);
      // approximated differential of encoder value
      for (int i = 0; i < 2; i++)
        enc_v.wheel[i] = (wheel_position[i][0] - wheel_position[i][1]) / Ts;
      enc_v.wheel2pole();
      // calculate est_vimated value
      float kt = 0.75f;
      float kr = 0.0f;
      est_v.tra = kt * (est_v.tra + accel[0] * Ts) + (1 - kt) * enc_v.tra;
      est_v.rot = kr * (est_v.rot + angular_accel[0] * Ts) + (1 - kr) * gyro[0];
      est_v.pole2wheel();
      // estimated accleration
      est_a.tra = accel.average();
      est_a.rot = angular_accel.average();
      est_a.pole2wheel();
      // calculate error integral
      for (int i = 0; i < 2; i++)
        integral.wheel[i] += (target_v.wheel[i] - est_v.wheel[i]) * Ts;
      integral.wheel2pole();
      // calculate pwm value
      pwm_value.tra = pidc_tra.calc(target_v.tra, est_v.tra, integral.tra,
                                    target_a.tra, est_a.tra);
      pwm_value.rot = pidc_rot.calc(target_v.rot, est_v.rot, integral.rot,
                                    target_a.rot, est_a.rot);
      pwm_value.pole2wheel();
      // drive the motors
      mt.drive(pwm_value.wheel[0], pwm_value.wheel[1]);

      // calculate odometry value
      position.theta += est_v.rot * Ts;
      position.x += est_v.tra * std::cos(position.theta) * Ts;
      position.y += est_v.tra * std::sin(position.theta) * Ts;
    }
  }
};
