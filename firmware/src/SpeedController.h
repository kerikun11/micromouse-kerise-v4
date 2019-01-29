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
  Position(const float pos[3]) : x(pos[0]), y(pos[1]), theta(pos[2]) {}

  void reset() {
    x = 0;
    y = 0;
    theta = 0;
  }
  void set(const float x = 0, const float y = 0, const float theta = 0) {
    this->x = x;
    this->y = y;
    this->theta = theta;
  }
  const Position rotate(const float angle) const {
    return Position(x * cos(angle) - y * sin(angle),
                    x * sin(angle) + y * cos(angle), theta);
  }
  float getNorm() const { return std::sqrt(x * x + y * y); }
  Position mirror_x() const { return Position(x, -y, -theta); }

  Position operator=(const Position &obj) {
    x = obj.x;
    y = obj.y;
    theta = obj.theta;
    return *this;
  }
  Position operator+() const { return Position(x, y, theta); }
  Position operator+(const Position &obj) const {
    return Position(x + obj.x, y + obj.y, theta + obj.theta);
  }
  Position operator+=(const Position &obj) {
    x += obj.x;
    y += obj.y;
    theta += obj.theta;
    return *this;
  }
  Position operator-() const { return Position(-x, -y, -theta); }
  Position operator-(const Position &obj) const {
    return Position(x - obj.x, y - obj.y, theta - obj.theta);
  }
  Position operator-=(const Position &obj) {
    x -= obj.x;
    y -= obj.y;
    theta -= obj.theta;
    return *this;
  }
  inline void print(const char *name = "Pos") {
    printf("%s: (%.1f,\t%.1f,\t%.3f)\n", name, x, y, theta);
  }
};

#define SPEED_CONTROLLER_TASK_PRIORITY 4
#define SPEED_CONTROLLER_STACK_SIZE 4096

#define SPEED_CONTROLLER_KP 0.6f
#define SPEED_CONTROLLER_KI 120.0f
#define SPEED_CONTROLLER_KD 0.03f

#define SPEED_CONTROLLER_PERIOD_US 1000

class SpeedController {
public:
  struct WheelParameter {
  public:
    float trans;    //< translation [mm]
    float rot;      //< rotation [rad]
    float wheel[2]; //< wheel position [mm], wheel[0]:left, wheel[1]:right
  public:
    WheelParameter() {}
    void pole2wheel() {
      wheel[0] = trans - MACHINE_ROTATION_RADIUS * rot;
      wheel[1] = trans + MACHINE_ROTATION_RADIUS * rot;
    }
    void wheel2pole() {
      rot = (wheel[1] - wheel[0]) / 2 / MACHINE_ROTATION_RADIUS;
      trans = (wheel[1] + wheel[0]) / 2;
    }
    void clear() {
      trans = 0;
      rot = 0;
      wheel[0] = 0;
      wheel[1] = 0;
    }
  };
  WheelParameter target;
  WheelParameter target_a;
  WheelParameter actual;
  WheelParameter enconly;
  WheelParameter acconly;
  WheelParameter proportional;
  WheelParameter integral;
  WheelParameter differential;
  float Kp = SPEED_CONTROLLER_KP;
  float Ki = SPEED_CONTROLLER_KI;
  float Kd = SPEED_CONTROLLER_KD;
  Position position;

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
      position.reset();
    enabled = true;
    printf("Speed Controller Enabled\n");
  }
  void disable() {
    enabled = false;
    delay(2);
    mt.free();
    printf("Speed Controller disabled\n");
  }
  void set_target(const float trans, const float rot, const float trans_a = 0,
                  const float rot_a = 0) {
    target.trans = trans;
    target.rot = rot;
    target.pole2wheel();
    target_a.trans = trans_a;
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
    target.clear();
    actual.clear();
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
      // xLastWakeTime = xTaskGetTickCount();
      if (enabled == false)
        continue; //< 有効でなければスルー

      // サンプリング終了まで待つ
      enc.samplingSemaphoreTake();
      imu.samplingSemaphoreTake();

      // 最新のデータの追加
      for (int i = 0; i < 2; i++)
        wheel_position[i].push(enc.position(i));
      accel.push(imu.accel.y);
      gyro.push(imu.gyro.z);
      angular_accel.push(imu.angular_accel);

      float sum_accel = 0.0f;
      for (int j = 0; j < acc_num - 1; j++)
        sum_accel += accel[j];

      for (int i = 0; i < 2; i++) {
        actual.wheel[i] =
            (wheel_position[i][0] - wheel_position[i][acc_num - 1]) /
            (acc_num - 1) * 1000000 / SPEED_CONTROLLER_PERIOD_US;
        enconly.wheel[i] = (wheel_position[i][0] - wheel_position[i][1]) *
                           1000000 / SPEED_CONTROLLER_PERIOD_US;
      }
      acconly.trans = sum_accel * SPEED_CONTROLLER_PERIOD_US / 1000000 / 2;
      enconly.wheel2pole();

      // calculate actual value
      actual.wheel2pole();
      actual.trans += sum_accel * SPEED_CONTROLLER_PERIOD_US / 1000000 / 2;
      actual.rot = imu.gyro.z;
      actual.pole2wheel();

      // calculate pid value
      for (int i = 0; i < 2; i++) {
        integral.wheel[i] += (target.wheel[i] - actual.wheel[i]) *
                             SPEED_CONTROLLER_PERIOD_US / 1000000;
        proportional.wheel[i] = target.wheel[i] - actual.wheel[i];
      }
      integral.wheel2pole();
      proportional.wheel2pole();
      differential.trans = target_a.trans - accel.average();
      differential.rot = target_a.rot - angular_accel.average();
      differential.pole2wheel();

      // calculate pwm value
      float pwm_value[2];
      for (int i = 0; i < 2; i++)
        pwm_value[i] = Kp * proportional.wheel[i] + Ki * integral.wheel[i] +
                       Kd * differential.wheel[i];
      mt.drive(pwm_value[0], pwm_value[1]);
      // fail safe
      const float pwm_em = Motor::MOTOR_DUTY_MAX * 2;
      if (fabs(pwm_value[0]) > pwm_em || fabs(pwm_value[1]) > pwm_em) {
        mt.emergency_stop();
        fan.drive(0);
      }

      // calculate odometry value
      position.theta += imu.gyro.z * SPEED_CONTROLLER_PERIOD_US / 1000000;
      position.x += enconly.trans * cos(position.theta) *
                    SPEED_CONTROLLER_PERIOD_US / 1000000;
      position.y += enconly.trans * sin(position.theta) *
                    SPEED_CONTROLLER_PERIOD_US / 1000000;
    }
  }
};
