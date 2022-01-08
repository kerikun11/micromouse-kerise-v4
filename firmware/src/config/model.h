/**
 * @file model.h
 * @brief マイクロマウスのモデル
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-04-20
 */
#pragma once

#include "ctrl/feedback_controller.h"
#include "ctrl/trajectory_tracker.h"

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
static constexpr float WheelDiameter = 12.65f;
static constexpr float CenterShift = 0.0f;
static constexpr float TailLength = 13.0f + CenterShift;
/* ToF */
static constexpr float tof_dist_offset = 16;
static constexpr float tof_dist_factor = 1.06f;
static constexpr float wall_fix_offset = 0; /*< 大きく: 前壁から遠く */
/* Reflector */
static constexpr float wall_attach_gain_Kp = 36.0f;
static constexpr float wall_attach_end = 0.5f;
static constexpr float wall_avoid_gain = 1e-3f;
/* Model */
static constexpr ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
        .K1 = ctrl::Polar(4000, 80),
        .T1 = ctrl::Polar(0.14, 0.08),
};
static constexpr ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
        .Kp = ctrl::Polar(0.001, 0.07),
        .Ki = ctrl::Polar(0.04, 4.0),
        .Kd = ctrl::Polar(0.0, 0.0),
};
static constexpr float turn_back_gain = 0.1;
/* Velocity Estimation IIR Filter gain */
static constexpr ctrl::Polar alpha = ctrl::Polar(1.0f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = 0.8f,
    .omega_n = 8.0f,
    .low_zeta = 0.5f,
    .low_b = 1e-2f,
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
static constexpr float tof_dist_factor = 1.07f;
static constexpr float wall_fix_offset = 8; /*< 大きく: 前壁から遠く */
/* Reflector */
static constexpr float wall_attach_gain_Kp = 24.0f;
static constexpr float wall_attach_end = 0.1f;
static constexpr float wall_avoid_gain = 0.003f;
/* Model */
static constexpr ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
        .K1 = ctrl::Polar(5789.0f, 1000.0f),
        .T1 = ctrl::Polar(0.12f, 0.48f), /*< 4 */
};
static constexpr ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
        .Kp = ctrl::Polar(0.0008f, 0.15f),
        .Ki = ctrl::Polar(0.1f, 6.0f), /*< 4 */
        .Kd = ctrl::Polar(0, 0),
};
static constexpr float turn_back_gain = 10.0f;
/* Velocity Estimation IIR Filter gain */
static constexpr ctrl::Polar alpha = ctrl::Polar(0.2f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
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
static constexpr float tof_dist_offset = 21; //< 大きいほど壁に近く
static constexpr float tof_dist_factor = 1.07f;
static constexpr float wall_fix_offset = 8; /*< 大きく: 前壁から遠く */
/* Reflector */
static constexpr float wall_attach_gain_Kp = 24.0f;
static constexpr float wall_attach_end = 0.1f;
static constexpr float wall_avoid_gain = 0.003f;
/* Model */
static constexpr ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
        .K1 = ctrl::Polar(5400, 100), //< 大きく：FF定常成分小さく
        .T1 = ctrl::Polar(0.18, 0.12), //< 大きく：FF立ち上がり成分大きく
};
static constexpr ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
        .Kp = ctrl::Polar(0.0004f, 0.03f),
        .Ki = ctrl::Polar(0.1f, 3.0f),
        .Kd = ctrl::Polar(0, 0),
};
static constexpr float turn_back_gain = 10.0f;
/* Velocity Estimation IIR Filter gain */
static constexpr ctrl::Polar alpha = ctrl::Polar(0.2f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
#if 0
    .zeta = 0.8f,
    .omega_n = 10.0f,
    .low_zeta = 1.0f,
    .low_b = 1e-3f,
#else
    .zeta = 0,
    .omega_n = 0,
    .low_zeta = 0,
    .low_b = 0,
#endif
};

#endif

}; // namespace model
