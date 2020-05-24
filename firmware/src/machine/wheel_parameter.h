#pragma once

#include "config/model.h"

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
