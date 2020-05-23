/**
 * @file model.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief マイクロマウスのモデル
 * @date 2020-04-20
 */
#pragma once

#include "feedback_controller.h"
#include "trajectory_tracker.h"

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

#define KERISE_SELECT 3

#if KERISE_SELECT == 5
/* KERISE v5 */
/* Machine Size Parameter */
static constexpr float RotationRadius = 29.0f / 2;
static constexpr float GearRatio = 1.0f;
static constexpr float WheelDiameter = 12.7f;
static constexpr float CenterShift = 0.0f;
static constexpr float TailLength = 13.0f + CenterShift;
/* ToF */
static constexpr float tof_dist_offset = 12;
/* Reflector */
static constexpr float wall_attach_gain_Kp = 12.0f;
static constexpr float wall_attach_gain_Ki = 0.05f;
static constexpr float wall_attach_end = 0.01f;
static constexpr float wall_avoid_gain = 1e-3f;
/* Model */
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
  .K1 = ctrl::Polar(5463.0f, 137.0f),
  .T1 = ctrl::Polar(float(0.1998 / 1.6), 0.1354f),
};
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
  .Kp = ctrl::Polar(0.0003f, 0.04f), .Ki = ctrl::Polar(0.06f, 1.0f),
  .Kd = ctrl::Polar(0, 0),
};
/* Velocity Estimation IIR Filter gain */
static constexpr struct ctrl::Polar alpha = ctrl::Polar(0.2f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr struct ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = 1.0f,
    .omega_n = 1.0f,
    .low_zeta = 0.5f,
    .low_b = 1e-3f,
};

#elif KERISE_SELECT == 4
/* Original KERISE v4 */
/* Machine Size Parameter */
static constexpr float RotationRadius = 15.0f;
static constexpr float GearRatio = (12.0f / 38.0f);
static constexpr float WheelDiameter = 12.67f;
static constexpr float CenterShift = 6.0f;
static constexpr float TailLength = 16.4f;
/* ToF */
static constexpr float tof_dist_offset = 21; //< 大きいほど壁に近く
/* Reflector */
static constexpr float wall_attach_gain_Kp = 240.0f;
static constexpr float wall_attach_gain_Ki = 1.0f;
static constexpr float wall_attach_end = 0.1f;
static constexpr float wall_avoid_gain = 0.003f;
/* Model */
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
  .K1 = ctrl::Polar(5789.0f, 1000.0f), .T1 = ctrl::Polar(0.12f, 0.48f), /*< 4 */
};
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
  .Kp = ctrl::Polar(0.0008f, 0.15f), .Ki = ctrl::Polar(0.1f, 6.0f), /*< 4 */
      .Kd = ctrl::Polar(0, 0),
};
/* Velocity Estimation IIR Filter gain */
static constexpr ctrl::Polar alpha = ctrl::Polar(0.2f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr struct ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = 0.8f,
    .omega_n = 18.0f,
    .low_zeta = 1.0f,
    .low_b = 1e-3f,
};

#elif KERISE_SELECT == 3
/* KERISE v4 Copy */
/* Machine Size Parameter */
static constexpr float RotationRadius = 15.0f;
static constexpr float GearRatio = (12.0f / 38.0f);
static constexpr float WheelDiameter = 12.95f;
static constexpr float CenterShift = 6.0f;
static constexpr float TailLength = 16.4f;
/* ToF */
static constexpr float tof_dist_offset = 8; //< 大きいほど壁に近く
/* Reflector */
static constexpr float wall_attach_gain_Kp = 240.0f;
static constexpr float wall_attach_gain_Ki = 0.5f;
static constexpr float wall_attach_end = 0.1f;
static constexpr float wall_avoid_gain = 0.003f;
/* Model */
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
  .K1 = ctrl::Polar(5833.0f, 66.72f), .T1 = ctrl::Polar(0.3694f, 0.1499f),
};
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
  .Kp = ctrl::Polar(5.3985e-03, 9.6774e-02),
  .Ki = ctrl::Polar(1.3552e-01, 1.6892e+00),
  .Kd = ctrl::Polar(7.7803e-06, 4.7964e-05),
};
/* Velocity Estimation IIR Filter gain */
static constexpr ctrl::Polar alpha = ctrl::Polar(0.05f, 0.2f);
/* Trajectory Tracking Gain */
static constexpr struct ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = 0.6f,
    .omega_n = 10.0f,
    .low_zeta = 1.0f,
    .low_b = 1e-2f,
};

#endif

}; // namespace model
