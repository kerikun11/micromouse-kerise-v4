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
static constexpr float M_PI = float(3.14159265358979323846);
#endif

namespace field {

/* Field Size Parameter */
static constexpr float SegWidthFull = float(90);
static constexpr float SegWidthDiag = float(127.2792206135786);
static constexpr float WallThickness = float(6);

}; // namespace field

namespace model {

#define KERISE_SELECT 3

#if KERISE_SELECT == 5
/* KERISE v5 */
/* Machine Size Parameter */
static constexpr float RotationRadius = float(29) / 2;
static constexpr float GearRatio = float(1);
static constexpr float WheelDiameter = float(12.7);
static constexpr float CenterShift = float(0);
static constexpr float TailLength = float(13) + CenterShift;
/* ToF */
static constexpr float tof_dist_offset = 12;
/* Reflector */
static constexpr float wall_attach_gain_Kp = float(12);
static constexpr float wall_attach_gain_Ki = float(0.05);
static constexpr float wall_attach_end = float(0.01);
static constexpr float wall_avoid_gain = float(1e-3);
/* Model */
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
  .K1 = ctrl::Polar(float(5463), float(137)),
  .T1 = ctrl::Polar(float(0.1998 / 1.6), float(0.1354)),
};
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
  .Kp = ctrl::Polar(float(0.0003), float(0.04)),
  .Ki = ctrl::Polar(float(0.06), float(1)), .Kd = ctrl::Polar(0, 0),
};
/* Estimated Velocity IIR Filter gain */
static constexpr struct ctrl::Polar alpha = ctrl::Polar(float(0.8), float(0));
/* Trajectory Tracking Gain */
static constexpr struct ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = float(1),
    .omega_n = float(1),
    .low_zeta = float(0.5),
    .low_b = float(1e-3),
};

#elif KERISE_SELECT == 4
/* Original KERISE v4 */
/* Machine Size Parameter */
static constexpr float RotationRadius = float(15);
static constexpr float GearRatio = (float(12) / float(38));
static constexpr float WheelDiameter = float(12.67);
static constexpr float CenterShift = float(6);
static constexpr float TailLength = float(16.4);
/* ToF */
static constexpr float tof_dist_offset = 21; //< 大きいほど壁に近く
/* Reflector */
static constexpr float wall_attach_gain_Kp = float(240);
static constexpr float wall_attach_gain_Ki = float(1);
static constexpr float wall_attach_end = float(0.1);
static constexpr float wall_avoid_gain = float(0.003);
/* Model */
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
  .K1 = ctrl::Polar(float(5789), float(1000)),
  .T1 = ctrl::Polar(float(0.12), float(0.48)), /*< 4 */
};
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
  .Kp = ctrl::Polar(float(0.0008), float(0.15)),
  .Ki = ctrl::Polar(float(0.1), float(6)), /*< 4 */
      .Kd = ctrl::Polar(0, 0),
};
/* Estimated Velocity IIR Filter gain */
static constexpr ctrl::Polar alpha = ctrl::Polar(float(0.8), float(0));
/* Trajectory Tracking Gain */
static constexpr struct ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = float(0.8),
    .omega_n = float(18),
    .low_zeta = float(1),
    .low_b = float(1e-3),
};

#elif KERISE_SELECT == 3
/* Copy KERISE v4 */
/* Machine Size Parameter */
static constexpr float RotationRadius = float(15);
static constexpr float GearRatio = (float(12) / float(38));
static constexpr float WheelDiameter = float(12.95);
static constexpr float CenterShift = float(6);
static constexpr float TailLength = float(16.4);
/* ToF */
static constexpr float tof_dist_offset = 8; //< 大きいほど壁に近く
/* Reflector */
static constexpr float wall_attach_gain_Kp = float(240);
static constexpr float wall_attach_gain_Ki = float(1);
static constexpr float wall_attach_end = float(0.1);
static constexpr float wall_avoid_gain = float(0.003);
/* Model */
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
  .K1 = ctrl::Polar(5789, float(1000)),
  .T1 = ctrl::Polar(float(0.12), 0.48), /*< 4 */
};
static constexpr struct ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
  // .Kp = ctrl::Polar(float(0.0006), float(0.1)),
  // .Ki = ctrl::Polar(float(0.1), float(3)), /*< 3 */
  .Kp = ctrl::Polar(float(0.0008), float(0.15)),
  .Ki = ctrl::Polar(float(0.1), float(6)), /*< 4 */
      .Kd = ctrl::Polar(0, 0),
};
/* Estimated Velocity IIR Filter gain */
static constexpr ctrl::Polar alpha = ctrl::Polar(float(0.8), float(0));
/* Trajectory Tracking Gain */
static constexpr struct ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = float(0.8),
    .omega_n = float(18),
    .low_zeta = float(1),
    .low_b = float(1e-3),
};

#endif

}; // namespace model
