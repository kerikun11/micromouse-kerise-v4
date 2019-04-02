#pragma once

#include "slalom.h"

using namespace ctrl;

static const auto SS_SL90 = ctrl::slalom::Shape(
    ctrl::Position(45, 45, 1.5708), ctrl::Position(39.9935, 40.0008, 1.5708),
    5.00647, 4.99916, 366.548);
static const auto SS_SR90 = ctrl::slalom::Shape(
    ctrl::Position(45, -45, -1.5708),
    ctrl::Position(39.9935, -40.0008, -1.5708), 5.00647, 4.99916, 366.548);
static const auto SS_FL45 = ctrl::slalom::Shape(
    ctrl::Position(90, 45, 0.785398), ctrl::Position(60.3485, 25, 0.785398),
    9.65155, 28.2843, 571.663);
static const auto SS_FR45 = ctrl::slalom::Shape(
    ctrl::Position(90, -45, -0.785398), ctrl::Position(60.3485, -25, -0.785398),
    9.65155, 28.2843, 571.663);
static const auto SS_FL90 = ctrl::slalom::Shape(
    ctrl::Position(90, 90, 1.5708), ctrl::Position(59.9905, 59.9979, 1.5708),
    30.0095, 30.0021, 549.854);
static const auto SS_FR90 = ctrl::slalom::Shape(
    ctrl::Position(90, -90, -1.5708),
    ctrl::Position(59.9905, -59.9979, -1.5708), 30.0095, 30.0021, 549.854);
static const auto SS_FL135 = ctrl::slalom::Shape(
    ctrl::Position(45, 90, 2.35619), ctrl::Position(33.1272, 80.0008, 2.35619),
    21.872, 14.1409, 506.758);
static const auto SS_FR135 = ctrl::slalom::Shape(
    ctrl::Position(45, -90, -2.35619),
    ctrl::Position(33.1272, -80.0008, -2.35619), 21.872, 14.1409, 506.758);
static const auto SS_FL180 =
    ctrl::slalom::Shape(ctrl::Position(0, 90, 3.14159),
                        ctrl::Position(0, 90, 3.14159), 30, 30, 557.649);
static const auto SS_FR180 =
    ctrl::slalom::Shape(ctrl::Position(0, -90, -3.14159),
                        ctrl::Position(0, -90, -3.14159), 30, 30, 557.649);
static const auto SS_FLV90 = ctrl::slalom::Shape(
    ctrl::Position(63.6396, 63.6396, 1.5708),
    ctrl::Position(41.9932, 42.0006, 1.5708), 21.6465, 21.639, 384.879);
static const auto SS_FRV90 = ctrl::slalom::Shape(
    ctrl::Position(63.6396, -63.6396, -1.5708),
    ctrl::Position(41.9932, -42.0006, -1.5708), 21.6465, 21.639, 384.879);
static const auto SS_FLS90 = ctrl::slalom::Shape(
    ctrl::Position(45, 45, 1.5708), ctrl::Position(44.9927, 45.0002, 1.5708),
    0.0072937, -0.000213623, 412.374);
static const auto SS_FRS90 =
    ctrl::slalom::Shape(ctrl::Position(45, -45, -1.5708),
                        ctrl::Position(44.9927, -45.0002, -1.5708), 0.0072937,
                        -0.000213623, 412.374);