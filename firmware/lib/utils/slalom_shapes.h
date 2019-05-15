#pragma once

#include "slalom.h"

using namespace ctrl;

static const auto SS_SL90 = ctrl::slalom::Shape(ctrl::Position(45, 45, 1.5708), ctrl::Position(44.0134, 44.0073, 1.5708), 0.986557, 0.992725, 265.761); //< T: 0.287448
static const auto SS_SR90 = ctrl::slalom::Shape(ctrl::Position(45, -45, -1.5708), ctrl::Position(44.0134, -44.0073, -1.5708), 0.986557, 0.992725, 265.761); //< T: 0.287448
static const auto SS_FL45 = ctrl::slalom::Shape(ctrl::Position(90, 45, 0.785398), ctrl::Position(74.8347, 31.0004, 0.785398), 1.16565, 19.7985, 425.272); //< T: 0.248641
static const auto SS_FR45 = ctrl::slalom::Shape(ctrl::Position(90, -45, -0.785398), ctrl::Position(74.8347, -31.0004, -0.785398), 1.16565, 19.7985, 425.272); //< T: 0.248641
static const auto SS_FL90 = ctrl::slalom::Shape(ctrl::Position(90, 90, 1.5708), ctrl::Position(70.0213, 70.0043, 1.5708), 19.9787, 19.9957, 422.846); //< T: 0.374537
static const auto SS_FR90 = ctrl::slalom::Shape(ctrl::Position(90, -90, -1.5708), ctrl::Position(70.0213, -70.0043, -1.5708), 19.9787, 19.9957, 422.846); //< T: 0.374537
static const auto SS_FL135 = ctrl::slalom::Shape(ctrl::Position(45, 90, 2.35619), ctrl::Position(35.2625, 85.0002, 2.35619), 14.7373, 7.07082, 375.888); //< T: 0.421351
static const auto SS_FR135 = ctrl::slalom::Shape(ctrl::Position(45, -90, -2.35619), ctrl::Position(35.2625, -85.0002, -2.35619), 14.7373, 7.07082, 375.888); //< T: 0.421351
static const auto SS_FL180 = ctrl::slalom::Shape(ctrl::Position(0, 90, 3.14159), ctrl::Position(0, 90, 3.14159), 24, 24, 412.408); //< T: 0.563056
static const auto SS_FR180 = ctrl::slalom::Shape(ctrl::Position(0, -90, -3.14159), ctrl::Position(0, -90, -3.14159), 24, 24, 412.408); //< T: 0.563056
static const auto SS_FLV90 = ctrl::slalom::Shape(ctrl::Position(63.6396, 63.6396, 1.5708), ctrl::Position(50.0153, 50.0078, 1.5708), 13.6243, 13.6319, 302.004); //< T: 0.370251
static const auto SS_FRV90 = ctrl::slalom::Shape(ctrl::Position(63.6396, -63.6396, -1.5708), ctrl::Position(50.0153, -50.0078, -1.5708), 13.6243, 13.6319, 302.004); //< T: 0.370251
static const auto SS_FLS90 = ctrl::slalom::Shape(ctrl::Position(45, 45, 1.5708), ctrl::Position(45.0139, 45.0082, 1.5708), -0.0138969, -0.00820541, 271.797); //< T: 0.279919
static const auto SS_FRS90 = ctrl::slalom::Shape(ctrl::Position(45, -45, -1.5708), ctrl::Position(45.0139, -45.0082, -1.5708), -0.0138969, -0.00820541, 271.797); //< T: 0.279919
