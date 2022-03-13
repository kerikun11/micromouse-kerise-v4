/**
 * @file wheel_parameter.h
 * @brief ホイールの回転量と並進・回転を相互変換する型を定義
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2022-03-13
 * @copyright Copyright 2022 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include "config/model.h"

struct WheelParameter {
 public:
  float tra;       //< translation [mm]
  float rot;       //< rotation [rad]
  float wheel[2];  //< wheel position [mm], wheel[0]:left, wheel[1]:right
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
