#pragma once

#include "slalom.h"

static const auto SS_SL90 =     ctrl::slalom::Shape(ctrl::Position(       45,        45,    1.5708), ctrl::Position(  44.0068,    44.001,    1.5708),  0.993221,  0.999035,    265.76); /*< T:  0.287496 [s] */
static const auto SS_SR90 =     ctrl::slalom::Shape(ctrl::Position(       45,       -45,   -1.5708), ctrl::Position(  44.0068,   -44.001,   -1.5708),  0.993221,  0.999035,    265.76); /*< T:  0.287496 [s] */
static const auto SS_FL45 =     ctrl::slalom::Shape(ctrl::Position(       90,        45,  0.785398), ctrl::Position(   74.835,        31,  0.785398),   1.16494,    19.799,   425.285); /*< T:  0.248639 [s] */
static const auto SS_FR45 =     ctrl::slalom::Shape(ctrl::Position(       90,       -45, -0.785398), ctrl::Position(   74.835,       -31, -0.785398),   1.16494,    19.799,   425.285); /*< T:  0.248639 [s] */
static const auto SS_FL90 =     ctrl::slalom::Shape(ctrl::Position(       90,        90,    1.5708), ctrl::Position(  70.0179,   69.9971,    1.5708),   19.9821,   20.0029,   422.868); /*< T:  0.374557 [s] */
static const auto SS_FR90 =     ctrl::slalom::Shape(ctrl::Position(       90,       -90,   -1.5708), ctrl::Position(  70.0179,  -69.9971,   -1.5708),   19.9821,   20.0029,   422.868); /*< T:  0.374557 [s] */
static const auto SS_FL135 =    ctrl::slalom::Shape(ctrl::Position(       45,        90,   2.35619), ctrl::Position(  33.1882,   80.0001,   2.35619),   21.8117,    14.142,   353.773); /*< T:  0.464962 [s] */
static const auto SS_FR135 =    ctrl::slalom::Shape(ctrl::Position(       45,       -90,  -2.35619), ctrl::Position(  33.1882,  -80.0001,  -2.35619),   21.8117,    14.142,   353.773); /*< T:  0.464962 [s] */
static const auto SS_FL180 =    ctrl::slalom::Shape(ctrl::Position(        0,        90,   3.14159), ctrl::Position(        0,        90,   3.14159),        24,        24,   412.408); /*< T:  0.563056 [s] */
static const auto SS_FR180 =    ctrl::slalom::Shape(ctrl::Position(        0,       -90,  -3.14159), ctrl::Position(        0,       -90,  -3.14159),        24,        24,   412.408); /*< T:  0.563056 [s] */
static const auto SS_FLV90 =    ctrl::slalom::Shape(ctrl::Position(  63.6396,   63.6396,    1.5708), ctrl::Position(  48.0118,   48.0008,    1.5708),   15.6278,   15.6388,   289.946); /*< T:  0.387836 [s] */
static const auto SS_FRV90 =    ctrl::slalom::Shape(ctrl::Position(  63.6396,  -63.6396,   -1.5708), ctrl::Position(  48.0118,  -48.0008,   -1.5708),   15.6278,   15.6388,   289.946); /*< T:  0.387836 [s] */
static const auto SS_FLS90 =    ctrl::slalom::Shape(ctrl::Position(       45,        45,    1.5708), ctrl::Position(  44.0068,    44.001,    1.5708),  0.993221,  0.999035,    265.76); /*< T:  0.287496 [s] */
static const auto SS_FRS90 =    ctrl::slalom::Shape(ctrl::Position(       45,       -45,   -1.5708), ctrl::Position(  44.0068,   -44.001,   -1.5708),  0.993221,  0.999035,    265.76); /*< T:  0.287496 [s] */
