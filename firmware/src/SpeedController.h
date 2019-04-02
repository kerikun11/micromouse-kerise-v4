#pragma once

#include "config/model.h"
#include "global.h"

#include "Accumulator.h"
#include "Position.h"
#include <cmath>

#define SPEED_CONTROLLER_TASK_PRIORITY 4
#define SPEED_CONTROLLER_STACK_SIZE 4096

namespace ctrl {

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
  struct Model {
    Polar K1;
    Polar T1;
    Polar Kp;
    Polar Ki;
    Polar Kd;
  };

public:
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
  void enable(const bool reset_position = true) {
    reset();
    if (reset_position)
      position.clear();
    enabled = true;
  }
  void disable() {
    enabled = false;
    delay(2);
    mt.free();
  }
  void set_target(const Polar dq, const Polar ddq) {
    ref_v = dq;
    ref_a = ddq;
  }
  void set_target(const float tra, const float rot, const float tra_a = 0,
                  const float rot_a = 0) {
    ref_v.tra = tra;
    ref_v.rot = rot;
    ref_a.tra = tra_a;
    ref_a.rot = rot_a;
  }
  void fix_position(const Position fix) { this->fix += fix; }

private:
  bool enabled = false;
  static const int acc_num = 8;
  Accumulator<float, acc_num> wheel_position[2];
  Accumulator<Polar, acc_num> accel;
  Position fix;

  void reset() {
    ref_v.clear();
    est_v.clear();
    e_int.clear();
    for (int i = 0; i < 2; i++)
      wheel_position[i].clear(enc.position(i));
    accel.clear(Polar(imu.accel.y, imu.angular_accel));
    fix.clear();
  }
  void task() {
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      /* 有効でなければスルー */
      if (enabled == false)
        continue;
      /* サンプリング終了まで待つ */
      enc.samplingSemaphoreTake();
      imu.samplingSemaphoreTake();
      /* 最新のデータの追加 */
      for (int i = 0; i < 2; i++)
        wheel_position[i].push(enc.position(i));
      accel.push(Polar(imu.accel.y, imu.angular_accel));
      /* approximated differential of encoder value */
      for (int i = 0; i < 2; i++)
        enc_v.wheel[i] = (wheel_position[i][0] - wheel_position[i][1]) / Ts;
      enc_v.wheel2pole();
      /* calculate estimated velocity value with complementary filter */
      Polar noisy_v = Polar(enc_v.tra, imu.gyro.z);
      Polar alpha = Polar(0.75f, 0.0f);
      est_v = alpha * (est_v + accel[0] * Ts) + (Polar(1, 1) - alpha) * noisy_v;
      /* estimated acceleration */
      est_a = accel[0];
      /* error integral */
      e_int += (ref_v - est_v) * Ts;
      /* feedforward */
      Polar K1 = Polar(5789, 49.74f);
      Polar T1 = Polar(0.2517f, 0.09089f);
      ff = (T1 * ref_a + ref_v) / K1;
      /* feedback */
      // Polar Kp = Polar(0, 0);
      // Polar Ki = Polar(0, 0);
      Polar Kp = Polar(0.001f, 0.04f);
      Polar Ki = Polar(0.1f, 3.0f);
      // Polar Kp = Polar(0.0f, 0.04f);
      // Polar Ki = Polar(0.0f, 3.0f);
      Polar Kd = Polar(0, 0);
      fbp = Kp * (ref_v - est_v);
      fbi = Ki * e_int;
      fb = Kp * (ref_v - est_v) + Ki * e_int + Kd * (ref_a - est_a);
      /* calculate pwm value */
      pwm_value = ff + fb;
      float pwm_value_L = pwm_value.tra - pwm_value.rot / 2;
      float pwm_value_R = pwm_value.tra + pwm_value.rot / 2;
      /* drive the motors */
      mt.drive(pwm_value_L, pwm_value_R);
      /* estimates slip angle */
      // const float k = 0.001f;
      const float k = 0.0f;
      const float slip_angle = k * est_v.tra * est_v.rot;
      /* calculate odometry value */
      position.th += est_v.rot * Ts;
      position.x += est_v.tra * std::cos(position.th + slip_angle) * Ts;
      position.y += est_v.tra * std::sin(position.th + slip_angle) * Ts;
      /* Fix Position */
      const float delta = 0.3;
      if (fix.x > delta) {
        position.x += delta;
        fix.x -= delta;
      } else if (fix.x < -delta) {
        position.x -= delta;
        fix.x += delta;
      }
      if (fix.y > delta) {
        position.y += delta;
        fix.y -= delta;
      } else if (fix.y < -delta) {
        position.y -= delta;
        fix.y += delta;
      }
    }
  }
};

}; // namespace ctrl
