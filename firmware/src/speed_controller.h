#pragma once

#include "config/model.h"
#include "global.h"

#include <accumulator.h>
#include <feedback_controller.h>
#include <freertospp/semphr.h>
#include <polar.h>
#include <pose.h>

#include <atomic>

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
  void clear() { tra = rot = wheel[0] = wheel[1] = 0; }
};

class SpeedController {
public:
  static constexpr const float Ts = 1e-3f;

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
  static constexpr int acc_num = 4;
  Accumulator<float, acc_num> wheel_position[2];
  Accumulator<ctrl::Polar, acc_num> accel;

public:
  SpeedController(const ctrl::FeedbackController<ctrl::Polar>::Model &M,
                  const ctrl::FeedbackController<ctrl::Polar>::Gain &G)
      : M(M), G(G), fbc(M, G) {
    reset();
  }
  bool init() {
    const UBaseType_t Priority = 5;
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "SC", 4096, this, Priority, NULL);
    return true;
  }
  void enable() {
    reset();
    enable_requested = true;
  }
  void disable() { disable_requested = false; }
  void sampling_sync() const { sampling_end_semaphore.take(); }
  void set_target(float v_tra, float v_rot, float a_tra = 0, float a_rot = 0) {
    ref_v.tra = v_tra, ref_v.rot = v_rot, ref_a.tra = a_tra, ref_a.rot = a_rot;
    reference_set_semaphore.take(0);
    drive();
  }
  void fix_pose(const ctrl::Pose &fix) { this->fix += fix; }
  void imu_calibration(const bool wait_for_end = true) {
    imu_calibration_requested = true;
    // calibration will be conducted in background task()
    while (wait_for_end && imu_calibration_requested)
      vTaskDelay(pdMS_TO_TICKS(1));
  }

private:
  std::atomic_bool enable_requested{false};
  std::atomic_bool disable_requested{false};
  std::atomic_bool imu_calibration_requested{false};
  freertospp::Semaphore sampling_end_semaphore;
  freertospp::Semaphore reference_set_semaphore;
  ctrl::Pose fix;

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool enabled = false;
    while (1) {
      /* sampling */
      update_samples();
      update_estimator();
      update_odometry();
      update_fix();
      /* notify end sampling */
      sampling_end_semaphore.give();
      /* give reference */
      reference_set_semaphore.give();
      /* sync */
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      /* request management */
      if (imu_calibration_requested) {
        mt.free();
        imu.calibration();
        imu_calibration_requested = false;
        xLastWakeTime = xTaskGetTickCount();
      }
      if (enable_requested)
        enabled = true;
      if (disable_requested)
        enabled = false, mt.free();
      /* if reference not changed */
      if (enabled && reference_set_semaphore.take(0))
        drive();
    }
  }
  void reset() {
    ref_v.clear();
    ref_a.clear();
    est_p.clear();
    est_v.clear();
    est_a.clear();
    enc_v.clear();
    for (int i = 0; i < 2; i++)
      wheel_position[i].clear(enc.get_position(i));
    accel.clear(ctrl::Polar(imu.accel.y, imu.angular_accel));
    fix.clear();
    fbc.reset();
  }
  void update_samples() {
    /* wait for end sampling */
    imu.update();
    enc.update();
    wd.update();
    /* add new samples */
    for (int i = 0; i < 2; i++)
      wheel_position[i].push(enc.get_position(i));
    accel.push(ctrl::Polar(imu.accel.y, imu.angular_accel));
  }
  void update_estimator() {
    /* calculate differential of encoder value */
    for (int i = 0; i < 2; i++)
      enc_v.wheel[i] = (wheel_position[i][0] - wheel_position[i][1]) / Ts;
    enc_v.wheel2pole();
    /* calculate estimated velocity value with complementary filter */
    const ctrl::Polar v_low = ctrl::Polar(enc_v.tra, imu.gyro.z);
    const ctrl::Polar v_high = est_v + accel[0] * float(Ts);
    const ctrl::Polar alpha = model::alpha;
    est_v = alpha * v_low + (ctrl::Polar(1, 1) - alpha) * v_high;
    /* estimated acceleration */
    est_a = accel[0];
  }
  void update_odometry() {
    /* estimates slip angle */
    // const float k = 0.01f;
    const float k = 0.0f;
    const float slip_angle = k * ref_v.tra * ref_v.rot / 1000;
    /* calculate odometry value */
    est_p.th += imu.gyro.z * Ts;
    est_p.x += enc_v.tra * std::cos(est_p.th + slip_angle) * Ts;
    est_p.y += enc_v.tra * std::sin(est_p.th + slip_angle) * Ts;
  }
  void update_fix() {
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
  void drive() {
    /* calculate pwm value */
    const auto pwm_value = fbc.update(ref_v, est_v, ref_a, est_a, Ts);
    const float pwm_value_L = pwm_value.tra - pwm_value.rot / 2;
    const float pwm_value_R = pwm_value.tra + pwm_value.rot / 2;
    /* drive the motors */
    mt.drive(pwm_value_L, pwm_value_R);
  }
};
