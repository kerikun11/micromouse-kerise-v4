#pragma once

#include "Position.h"
#include "ctrl/FeedbackController.h"

#ifndef M_PI
static constexpr float M_PI = 3.14159265358979323846f;
#endif

namespace field {

/* Field Size Parameter */
static constexpr float SegWidthFull = 90.0f;
static constexpr float SegWidthDiag = 127.2792206135786f;
static constexpr float WallThickness = 6.0f;

}; // namespace field

namespace model {

/* Machine Size Parameter */
static constexpr float RotationRadius = 15.0f;
static constexpr float GearRatio = (12.0f / 38.0f);
static constexpr float WheelDiameter = 12.67f;
static constexpr float CenterShift = 5.0f;
static constexpr float TailLength = 16.4f + CenterShift;

#if 1
/* Model */
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
  .K1 = ctrl::Polar(5789, 49.74f), .T1 = ctrl::Polar(0.2517f, 0.09089f),
};
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
  .Kp = ctrl::Polar(0.001f, 0.04f), .Ki = ctrl::Polar(0.2f, 6.0f),
  .Kd = ctrl::Polar(0, 0),
};
/* Trajectory Tracking Gain */
static constexpr float tt_gain = 10.0f;
#else
/* Model */
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
  .K1 = ctrl::Polar(5906, 84.01f), .T1 = ctrl::Polar(0.3235f, 0.1798),
};
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
  .Kp = ctrl::Polar(0.001f, 0.04f), .Ki = ctrl::Polar(0.2f, 6.0f),
  .Kd = ctrl::Polar(0, 0),
};
/* Trajectory Tracking Gain */
static constexpr float tt_gain = 5.0f;
#endif

}; // namespace model
