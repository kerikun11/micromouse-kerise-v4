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
  class Polar {
  public:
    float tra; //< tralation [mm]
    float rot; //< rotation [rad]
  public:
    Polar(const float tra = 0, const float rot = 0) : tra(tra), rot(rot) {}
    Polar(const Polar &o) : tra(o.tra), rot(o.rot) {}
    const Polar &operator=(const Polar &o) {
      tra = o.tra;
      rot = o.rot;
      return *this;
    }
    const Polar &operator+=(const Polar &o) {
      tra += o.tra;
      rot += o.rot;
      return *this;
    }
    const Polar &operator*=(const Polar &o) {
      tra *= o.tra;
      rot *= o.rot;
      return *this;
    }
    const Polar &operator/=(const Polar &o) {
      tra /= o.tra;
      rot /= o.rot;
      return *this;
    }
    const Polar operator+(const Polar &o) const {
      return Polar(tra + o.tra, rot + o.rot);
    }
    const Polar operator-(const Polar &o) const {
      return Polar(tra - o.tra, rot - o.rot);
    }
    const Polar operator*(const Polar &o) const {
      return Polar(tra * o.tra, rot * o.rot);
    }
    const Polar operator/(const Polar &o) const {
      return Polar(tra / o.tra, rot / o.rot);
    }
    const Polar operator+(const float &k) const {
      return Polar(tra + k, rot + k);
    }
    const Polar operator-(const float &k) const {
      return Polar(tra - k, rot - k);
    }
    const Polar operator*(const float &k) const {
      return Polar(tra * k, rot * k);
    }
    const Polar operator/(const float &k) const {
      return Polar(tra / k, rot / k);
    }
    void clear() { tra = rot = 0; }
  };
  WheelParameter target_v;
  WheelParameter target_a;
  WheelParameter est_v;
  WheelParameter est_a;
  WheelParameter enc_v;
  Position position;

  Polar Kp;
  Polar Ki;
  Polar Kd;
  Polar e_int;

public:
  SpeedController() {
    enabled = false;
    reset();
    Kp = Polar(0.0006f, 0.0006f);
    Ki = Polar(0.12f, 0.12f);
    Kd = Polar(0.00003f, 0.00003f);
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
    e_int.clear();
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
      // ref and out
      Polar ref_v(target_v.tra, target_v.rot);
      Polar ref_a(target_a.tra, target_a.rot);
      Polar out_v(est_v.tra, est_v.rot);
      Polar out_a(est_a.tra, est_a.rot);
      // feedforward
      Polar K = Polar(6465, 54.96f);
      Polar T1 = Polar(0.264f, 0.08372f);
      Polar ff = (T1 * ref_a + ref_v) / K;
      // feedback
      e_int += (ref_v - out_v) * Ts;
      Polar fb = Kp * (ref_v - out_v) + Ki * e_int + Kd * (ref_a - out_a);

      // calculate pwm value

      // Polar pwm_value = ff + fb;
      // float pwm_value_L = pwm_value.tra - pwm_value.rot / 2;
      // float pwm_value_R = pwm_value.tra + pwm_value.rot / 2;
      float r = MACHINE_ROTATION_RADIUS;
      float pwm_value_L = ff.tra - ff.rot / 2 + fb.tra - fb.rot * r;
      float pwm_value_R = ff.tra + ff.rot / 2 + fb.tra + fb.rot * r;

      // drive the motors
      mt.drive(pwm_value_L, pwm_value_R);

      // calculate odometry value
      position.theta += est_v.rot * Ts;
      position.x += est_v.tra * std::cos(position.theta) * Ts;
      position.y += est_v.tra * std::sin(position.theta) * Ts;
    }
  }
};
