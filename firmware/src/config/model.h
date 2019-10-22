#pragma once

#include "Position.h"
#include "TrajectoryTracker.h"
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

#define KERISE_SELECT 4

#if KERISE_SELECT == 5
/* KERISE v5 */
/* Machine Size Parameter */
static constexpr float RotationRadius = 29.0f / 2;
static constexpr float GearRatio = 1.0f;
static constexpr float WheelDiameter = 12.60f;
static constexpr float CenterShift = 0.0f;
static constexpr float TailLength = 13.0f + CenterShift;
/* ToF */
static constexpr float tof_dist_offset = 12;
const float wall_attach_gain_Kp = 6.0f;
const float wall_attach_gain_Ki = 0.0f;
/* Model */
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
  .K1 = ctrl::Polar(5400, 90.0f), .T1 = ctrl::Polar(0.06f, 0.06f),
};
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
  // .Kp = ctrl::Polar(0.0003f, 0.04f), .Ki = ctrl::Polar(0.01f, 3.0f),
  .Kp = ctrl::Polar(0.0002f, 0.03f), .Ki = ctrl::Polar(0.03f, 3.0f),
  .Kd = ctrl::Polar(0, 0),
};
/* Estimated Velocity IIR Filter gain */
static constexpr struct ctrl::Polar alpha = ctrl::Polar(0.5f, 0.0f);
/* Trajectory Tracking Gain */
static constexpr float tt_gain = 10.0f;

#elif KERISE_SELECT == 4
/* Original KERISE v4 */
/* Machine Size Parameter */
static constexpr float RotationRadius = 15.0f;
static constexpr float GearRatio = (12.0f / 38.0f);
static constexpr float WheelDiameter = 12.67f;
static constexpr float CenterShift = 5.0f;
static constexpr float TailLength = 16.4f + CenterShift;
/* ToF */
static constexpr float tof_dist_offset = 23; //< 大きいほど壁に近く
const float wall_attach_gain_Kp = 180.0f;
const float wall_attach_gain_Ki = 0.0f;
/* Model */
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
  .K1 = ctrl::Polar(5789, 49.74f), .T1 = ctrl::Polar(0.12f, 0.08f),
};
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
  .Kp = ctrl::Polar(0.0006f, 0.1f), .Ki = ctrl::Polar(0.1f, 3.0f),
  .Kd = ctrl::Polar(0, 0),
};
/* Estimated Velocity IIR Filter gain */
static constexpr ctrl::Polar alpha = ctrl::Polar(0.75f, 0.0f);
/* Trajectory Tracking Gain */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain;

#elif KERISE_SELECT == 3
/* Copy KERISE v4 */
/* Machine Size Parameter */
static constexpr float RotationRadius = 15.0f;
static constexpr float GearRatio = (12.0f / 38.0f);
static constexpr float WheelDiameter = 12.80f;
static constexpr float CenterShift = 5.0f;
static constexpr float TailLength = 16.4f + CenterShift;
/* ToF */
static constexpr float tof_dist_offset = 20;
const float wall_attach_gain_Kp = 120.0f;
const float wall_attach_gain_Ki = 0.0f;
/* Model */
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
  .K1 = ctrl::Polar(5789, 49.74f), .T1 = ctrl::Polar(0.12f, 0.08f),
};
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
  .Kp = ctrl::Polar(0.0006f, 0.1f), .Ki = ctrl::Polar(0.1f, 3.0f),
  .Kd = ctrl::Polar(0, 0),
};
/* Estimated Velocity IIR Filter gain */
static constexpr struct ctrl::Polar alpha = ctrl::Polar(0.75f, 0.0f);
/* Trajectory Tracking Gain */
static constexpr float tt_gain = 15.0f;

#endif

}; // namespace model
