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
  Polar ref_v;
  Polar ref_a;
  Polar est_v;
  Polar est_a;
  Polar e_int;
  WheelParameter enc_v;
  Position position;
  Polar ff;
  Polar fb;
  Polar fbp;
  Polar fbi;
  Polar pwm_value;

public:
  SpeedController() {
    enabled = false;
    reset();
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
    ref_v.tra = tra;
    ref_v.rot = rot;
    ref_a.tra = tra_a;
    ref_a.rot = rot_a;
  }

private:
  bool enabled = false;
  static const int acc_num = 8;
  Accumulator<float, acc_num> wheel_position[2];
  Accumulator<Polar, acc_num> accel;

  void reset() {
    ref_v.clear();
    est_v.clear();
    e_int.clear();
    for (int i = 0; i < 2; i++)
      wheel_position[i].clear(enc.position(i));
    accel.clear(Polar(imu.accel.y, imu.angular_accel));
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
      accel.push(Polar(imu.accel.y, imu.angular_accel));
      // approximated differential of encoder value
      for (int i = 0; i < 2; i++)
        enc_v.wheel[i] = (wheel_position[i][0] - wheel_position[i][1]) / Ts;
      enc_v.wheel2pole();
      // calculate estimated velocity value with complementary filter
      Polar noisy_v = Polar(enc_v.tra, imu.gyro.z);
      Polar alpha = Polar(0.75f, 0.0f);
      est_v = alpha * (est_v + accel[0] * Ts) + (Polar(1, 1) - alpha) * noisy_v;
      // estimated acceleration
      est_a = accel[0];
      // error integral
      e_int += (ref_v - est_v) * Ts;
      // feedforward
      Polar K1 = Polar(6465, 54.96f);
      Polar T1 = Polar(0.264f, 0.08372f);
      ff = (T1 * ref_a + ref_v) / K1;
      // feedback
      // Polar Kp = Polar(0.002f, 0.018919f);
      // Polar Ki = Polar(0.1f, 0.321020f);
      Polar Kp = Polar(0.002f, 0.002f);
      Polar Ki = Polar(0.1f, 2.0f);
      Polar Kd = Polar(0, 0);
      fbp = Kp * (ref_v - est_v);
      fbi = Ki * e_int;
      fb = Kp * (ref_v - est_v) + Ki * e_int + Kd * (ref_a - est_a);
      // calculate pwm value
      pwm_value = ff + fb;
      float pwm_value_L = pwm_value.tra - pwm_value.rot / 2;
      float pwm_value_R = pwm_value.tra + pwm_value.rot / 2;
      // drive the motors
      mt.drive(pwm_value_L, pwm_value_R);

      // calculate odometry value
      position.theta += est_v.rot * Ts;
      position.x += est_v.tra * std::cos(position.theta) * Ts;
      position.y += est_v.tra * std::sin(position.theta) * Ts;
    }
  }
};
