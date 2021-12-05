/**
 * @file speed_controller.h
 * @brief Speed Controller
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include "config/wheel_parameter.h"
#include "machine/global.h"

#include <freertospp/semphr.h>

#include <ctrl/accumulator.h>
#include <ctrl/feedback_controller.h>
#include <ctrl/polar.h>
#include <ctrl/pose.h>

#include <atomic>

class SpeedController {
public:
  static constexpr const float Ts = 1e-3f;

public:
  ctrl::Polar ref_v;
  ctrl::Polar ref_a;
  ctrl::Polar est_v;
  ctrl::Polar est_a;
  ctrl::Pose est_p;
  WheelParameter enc_v;
  ctrl::FeedbackController<ctrl::Polar> fbc;
  static constexpr int acc_num = 4;
  ctrl::Accumulator<float, acc_num> wheel_position[2];
  ctrl::Accumulator<ctrl::Polar, acc_num> accel;

private:
  Motor &mt;
  IMU &imu;
  Encoder &enc;
  Fan &fan;

public:
  SpeedController(const ctrl::FeedbackController<ctrl::Polar>::Model &M,
                  const ctrl::FeedbackController<ctrl::Polar>::Gain &G,
                  Motor &mt, IMU &imu, Encoder &enc, Fan &fan)
      : fbc(M, G), mt(mt), imu(imu), enc(enc), fan(fan) {
    reset();
  }
  bool init() {
    xTaskCreatePinnedToCore(
        [](void *arg) { static_cast<decltype(this)>(arg)->task(); },
        "SpeedCtrl", 4096, this, 5, NULL, PRO_CPU_NUM);
    return true;
  }
  void enable() {
    // app_logi << "enable" << std::endl;
    reset();
    drive_enabled = true;
  }
  void disable() {
    // app_logi << "disable" << std::endl;
    drive_enabled = false;
    sampling_sync();
    sampling_sync();
    mt.free();
    fan.drive(0);
  }
  void set_target(float v_tra, float v_rot, float a_tra = 0, float a_rot = 0) {
    ref_v.tra = v_tra, ref_v.rot = v_rot, ref_a.tra = a_tra, ref_a.rot = a_rot;
  }
  void fix_pose(const ctrl::Pose &fix) { this->fix += fix; }
  void sampling_sync() const { //
    data_ready_semaphore.take();
  }

private:
  std::atomic_bool drive_enabled{false};
  freertospp::Semaphore data_ready_semaphore;
  ctrl::Pose fix;

  void task() {
    while (1) {
      // const int us_start = esp_timer_get_time();
      /* sync */
      imu.sampling_sync();
      enc.sampling_sync();
      /* sampling */
      update_samples();
      update_estimator();
      update_odometry();
      update_fix();
      /* notify */
      data_ready_semaphore.give();
      /* drive */
      if (drive_enabled)
        drive();
      // const int us_end = esp_timer_get_time();
      /* debug */
      // if (us_end - us_start > 1500 && drive_enabled) {
      //   bz.play(Buzzer::SHORT9);
      //   app_logw << "sampling overtime: " << int(us_end - us_start)
      //            << std::endl;
      // }
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
