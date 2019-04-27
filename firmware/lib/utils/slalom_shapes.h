#pragma once

#include "slalom.h"

using namespace ctrl;

static const auto SS_SL90 = ctrl::slalom::Shape(
    ctrl::Position(45, 45, 1.5708), ctrl::Position(44.0134, 44.0073, 1.5708),
    0.986569, 0.992725, 265.761);
static const auto SS_SR90 = ctrl::slalom::Shape(
    ctrl::Position(45, -45, -1.5708),
    ctrl::Position(44.0134, -44.0073, -1.5708), 0.986569, 0.992725, 265.761);
static const auto SS_FL45 = ctrl::slalom::Shape(
    ctrl::Position(90, 45, 0.785398),
    ctrl::Position(57.9372, 24.0003, 0.785398), 11.0631, 29.698, 329.242);
static const auto SS_FR45 = ctrl::slalom::Shape(
    ctrl::Position(90, -45, -0.785398),
    ctrl::Position(57.9372, -24.0003, -0.785398), 11.0631, 29.698, 329.242);
static const auto SS_FL90 = ctrl::slalom::Shape(
    ctrl::Position(90, 90, 1.5708), ctrl::Position(60.0184, 60.0047, 1.5708),
    29.9816, 29.9953, 362.433);
static const auto SS_FR90 = ctrl::slalom::Shape(
    ctrl::Position(90, -90, -1.5708),
    ctrl::Position(60.0184, -60.0047, -1.5708), 29.9816, 29.9953, 362.433);
static const auto SS_FL135 = ctrl::slalom::Shape(
    ctrl::Position(45, 90, 2.35619), ctrl::Position(33.1881, 80.0014, 2.35619),
    21.8106, 14.1402, 353.771);
static const auto SS_FR135 = ctrl::slalom::Shape(
    ctrl::Position(45, -90, -2.35619),
    ctrl::Position(33.1881, -80.0014, -2.35619), 21.8106, 14.1402, 353.771);
static const auto SS_FL180 =
    ctrl::slalom::Shape(ctrl::Position(0, 90, 3.14159),
                        ctrl::Position(0, 90, 3.14159), 30, 30, 412.408);
static const auto SS_FR180 =
    ctrl::slalom::Shape(ctrl::Position(0, -90, -3.14159),
                        ctrl::Position(0, -90, -3.14159), 30, 30, 412.408);
static const auto SS_FLV90 = ctrl::slalom::Shape(
    ctrl::Position(63.6396, 63.6396, 1.5708),
    ctrl::Position(42.0131, 42.0035, 1.5708), 21.6265, 21.6361, 253.702);
static const auto SS_FRV90 = ctrl::slalom::Shape(
    ctrl::Position(63.6396, -63.6396, -1.5708),
    ctrl::Position(42.0131, -42.0035, -1.5708), 21.6265, 21.6361, 253.702);
static const auto SS_FLS90 = ctrl::slalom::Shape(
    ctrl::Position(45, 45, 1.5708), ctrl::Position(45.0139, 45.0082, 1.5708),
    -0.0138931, -0.00820541, 271.797);
static const auto SS_FRS90 =
    ctrl::slalom::Shape(ctrl::Position(45, -45, -1.5708),
                        ctrl::Position(45.0139, -45.0082, -1.5708), -0.0138931,
                        -0.00820541, 271.797);