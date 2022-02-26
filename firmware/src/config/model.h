/**
 * @file model.h
 * @brief マイクロマウスのモデル
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-04-20
 */
#pragma once

#include "ctrl/feedback_controller.h"
#include "ctrl/trajectory_tracker.h"

/* Math Constants */
static constexpr float PI = 3.14159265358979323846f;

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
static constexpr uint64_t MAC_ID = 0xD866'5A1D'A0D8; //< efuse 48 bit MAC
/* Machine Size Parameter */
static constexpr float RotationRadius = 29.0f / 2;
static constexpr float GearRatio = 1.0f;
static constexpr float WheelDiameter = 12.72f; //< 径を大きく：進行距離を短く
static constexpr float CenterOffsetY = 0.0f;
static constexpr float TailLength = 13.0f;
/* ToF */
static constexpr float tof_raw_range_90 = 75;
static constexpr float tof_raw_range_180 = 160;
static constexpr float wall_fix_offset = -5; /*< 大きく: 前壁に近く */
/* Reflector */
static constexpr float front_front_wall_attach_gain_Kp = 36.0f;
static constexpr float front_front_wall_attach_end = 0.5f;
static constexpr float wall_avoid_gain = 1e-4f;
static constexpr float wall_fix_theta_gain = 1e-9f;
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
static constexpr float turn_back_gain = 10.0f;
/* Velocity Estimation IIR Filter gain */
static constexpr ctrl::Polar velocity_filter_alpha = ctrl::Polar(1.0f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = 0.8f,
    .omega_n = 8.0f,
    .low_zeta = 0.5f,
    .low_b = 1e-2f,
};

#elif KERISE_SELECT == 4
/* Original KERISE v4 */
static constexpr uint64_t MAC_ID = 0x080C'401D'A0D8; //< efuse 48 bit MAC
/* Machine Size Parameter */
static constexpr float RotationRadius = 15.0f;
static constexpr float GearRatio = (12.0f / 38.0f);
static constexpr float WheelDiameter = 12.67f; //< 径を大きく：進行距離を短く
static constexpr float CenterOffsetY = 8.0f;
static constexpr float TailLength = 16.4f;
/* ToF */
static constexpr float tof_raw_range_90 = 69;
static constexpr float tof_raw_range_180 = 154;
static constexpr float wall_fix_offset = -5; /*< 大きく: 前壁に近く */
/* Reflector */
static constexpr float front_wall_attach_gain_Kp = 24.0f;
static constexpr float front_wall_attach_end = 0.1f;
static constexpr float wall_avoid_gain = 0.003f;
static constexpr float wall_fix_theta_gain = 1e-9f;
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
static constexpr ctrl::Polar velocity_filter_alpha = ctrl::Polar(0.2f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = 0.8f,
    .omega_n = 16.0f,
    .low_zeta = 1.0f,
    .low_b = 1e-3f,
};

#elif KERISE_SELECT == 3
/* KERISE v4 Copy */
static constexpr uint64_t MAC_ID = 0x807F'631D'A0D8; //< efuse 48 bit MAC
/* Machine Size Parameter */
static constexpr float RotationRadius = 15.0f;
static constexpr float GearRatio = (12.0f / 38.0f);
static constexpr float WheelDiameter = 12.96f; //< 径を大きく：進行距離を短く
static constexpr float CenterOffsetY = 8.0f;
static constexpr float TailLength = 16.4f;
/* ToF */
static constexpr float tof_raw_range_90 = 69;
static constexpr float tof_raw_range_180 = 154;
static constexpr float wall_fix_offset = -5; /*< 大きく: 前壁に近く */
/* Reflector */
static constexpr float front_wall_attach_gain_Kp = 24.0f;
static constexpr float front_wall_attach_end = 0.1f;
static constexpr float wall_avoid_gain = 0.003f;
static constexpr float wall_fix_theta_gain = 1e-9f;
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
static constexpr ctrl::Polar velocity_filter_alpha = ctrl::Polar(0.2f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
#if 1
    .zeta = 0.6f,
    .omega_n = 6.0f,
    .low_zeta = 0.5f,
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
