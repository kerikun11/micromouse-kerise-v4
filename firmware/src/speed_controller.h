#pragma once

#include "accumulator.h"
#include "config/model.h"
#include "global.h"

#include "feedback_controller.h"
#include "polar.h"
#include "pose.h"

#define SPEED_CONTROLLER_TASK_PRIORITY 4
#define SPEED_CONTROLLER_STACK_SIZE 4096

struct WheelParameter {
public:
  float tra;      //< translation [mm]
  float rot;      //< rotation [rad]
  float wheel[2]; //< wheel position [mm], wheel[0]:left, wheel[1]:right
public:
  WheelParameter() { clear(); }
  void pole2wheel() {
    wheel[0] = tra - model::RotationRadius * rot;
    wheel[1] = tra + model::RotationRadius * rot;
  }
  void wheel2pole() {
    rot = (wheel[1] - wheel[0]) / 2 / model::RotationRadius;
    tra = (wheel[1] + wheel[0]) / 2;
  }
  void clear() {
    tra = 0;
    rot = 0;
    wheel[0] = 0;
    wheel[1] = 0;
  }
};

class SpeedController {
public:
  static constexpr float Ts = float(1e-3);

public:
  ctrl::Polar ref_v;
  ctrl::Polar ref_a;
  ctrl::Polar est_v;
  ctrl::Polar est_a;
  WheelParameter enc_v;
  ctrl::Pose est_p;
  ctrl::FeedbackController<ctrl::Polar>::Model M;
  ctrl::FeedbackController<ctrl::Polar>::Gain G;
  ctrl::FeedbackController<ctrl::Polar> fbc;

public:
  SpeedController(const ctrl::FeedbackController<ctrl::Polar>::Model &M,
                  const ctrl::FeedbackController<ctrl::Polar>::Gain &G)
      : M(M), G(G), fbc(M, G) {
    enabled = false;
    reset();
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "SpeedController", SPEED_CONTROLLER_STACK_SIZE, this,
                SPEED_CONTROLLER_TASK_PRIORITY, NULL);
  }
  void enable() {
    reset();
    enabled = true;
  }
  void disable() {
    enabled = false;
    vTaskDelay(pdMS_TO_TICKS(2));
    mt.free();
  }
  void set_target(const float tra, const float rot, const float tra_a = 0,
                  const float rot_a = 0) {
    ref_v.tra = tra;
    ref_v.rot = rot;
    ref_a.tra = tra_a;
    ref_a.rot = rot_a;
  }
  void fix_pose(const ctrl::Pose fix) { this->fix += fix; }

private:
  volatile bool enabled = false;
  static constexpr int acc_num = 8;
  Accumulator<float, acc_num> wheel_position[2];
  Accumulator<ctrl::Polar, acc_num> accel;
  ctrl::Pose fix;

  void reset() {
    ref_v.clear();
    ref_a.clear();
    est_v.clear();
    est_a.clear();
    enc_v.clear();
    for (int i = 0; i < 2; i++)
      wheel_position[i].clear(enc.get_position(i));
    accel.clear(ctrl::Polar(imu.accel.y, imu.angular_accel));
    fix.clear();
    fbc.reset();
  }
  void update() {
    /* 最新のデータの追加 */
    for (int i = 0; i < 2; i++)
      wheel_position[i].push(enc.get_position(i));
    accel.push(ctrl::Polar(imu.accel.y, imu.angular_accel));
    /* approximated differential of encoder value */
    for (int i = 0; i < 2; i++)
      enc_v.wheel[i] = (wheel_position[i][0] - wheel_position[i][1]) / Ts;
    enc_v.wheel2pole();
    /* calculate estimated velocity value with complementary filter */
    const ctrl::Polar noisy_v = ctrl::Polar(enc_v.tra, imu.gyro.z);
    const ctrl::Polar alpha = model::alpha;
    est_v =
        alpha * (est_v + accel[0] * Ts) + (ctrl::Polar(1, 1) - alpha) * noisy_v;
    /* estimated acceleration */
    est_a = accel[0];
    /* calculate pwm value */
    const auto pwm_value = fbc.update(ref_v, est_v, ref_a, est_a, Ts);
    const float pwm_value_L = pwm_value.tra - pwm_value.rot / 2;
    const float pwm_value_R = pwm_value.tra + pwm_value.rot / 2;
    /* drive the motors */
    mt.drive(pwm_value_L, pwm_value_R);
    /* estimates slip angle */
    // const float k = float(0.01);
    const float k = float(0);
    const float slip_angle = k * ref_v.tra * ref_v.rot / 1000;
    /* calculate odometry value */
    est_p.th += imu.gyro.z * Ts;
    est_p.x += enc_v.tra * std::cos(est_p.th + slip_angle) * Ts;
    est_p.y += enc_v.tra * std::sin(est_p.th + slip_angle) * Ts;
    /* Fix Pose */
    const float delta = 0.2;
    if (fix.x > delta) {
      est_p.x += delta;
      fix.x -= delta;
    } else if (fix.x < -delta) {
      est_p.x -= delta;
      fix.x += delta;
    }
    if (fix.y > delta) {
      est_p.y += delta;
      fix.y -= delta;
    } else if (fix.y < -delta) {
      est_p.y -= delta;
      fix.y += delta;
    }
  }
  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      xLastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      /* 有効でなければスルー */
      if (enabled == false)
        continue;
      /* サンプリング終了まで待つ */
      enc.samplingSemaphoreTake();
      imu.samplingSemaphoreTake();
      /* 更新処理 */
      update();
    }
  }
};
