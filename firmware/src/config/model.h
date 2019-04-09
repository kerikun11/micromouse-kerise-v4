#pragma once

#ifndef M_PI
static constexpr float M_PI = 3.14159265358979323846f;
#endif

/* Machine Size Parameter */
#define MACHINE_ROTATION_RADIUS 15.0f
#define MACHINE_GEAR_RATIO (12.0f / 38.0f)
#define MACHINE_WHEEL_DIAMETER 12.67f
#define MACHINE_TAIL_LENGTH 16.4f

/* Field Size Parameter */
#define SEGMENT_WIDTH 90
#define SEGMENT_DIAGONAL_WIDTH 127.2792206135786f
#define WALL_THICKNESS 6.0f

/* Model */
#include "Position.h"
#include "ctrl/FeedbackController.h"
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
static constexpr float tt_gain = 5;
