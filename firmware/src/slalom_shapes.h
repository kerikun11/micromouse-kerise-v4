#pragma once

#include "slalom.h"

static const auto SS_S90L =     ctrl::slalom::Shape(ctrl::Position(       45,        45,    1.5708), ctrl::Position(  44.0068,    44.001,    1.5708),  0.993221,  0.999035,    265.76); /*< T:  0.287496 [s] */
static const auto SS_S90R =     ctrl::slalom::Shape(ctrl::Position(       45,       -45,   -1.5708), ctrl::Position(  44.0068,   -44.001,   -1.5708),  0.993221,  0.999035,    265.76); /*< T:  0.287496 [s] */
static const auto SS_F45L =     ctrl::slalom::Shape(ctrl::Position(       90,        45,  0.785398), ctrl::Position(  72.4217,        30,  0.785398),   2.57825,   21.2132,   411.566); /*< T:  0.257152 [s] */
static const auto SS_F45R =     ctrl::slalom::Shape(ctrl::Position(       90,       -45, -0.785398), ctrl::Position(  72.4217,       -30, -0.785398),   2.57825,   21.2132,   411.566); /*< T:  0.257152 [s] */
static const auto SS_F90L =     ctrl::slalom::Shape(ctrl::Position(       90,        90,    1.5708), ctrl::Position(  70.0179,   69.9971,    1.5708),   19.9821,   20.0029,   422.868); /*< T:  0.374557 [s] */
static const auto SS_F90R =     ctrl::slalom::Shape(ctrl::Position(       90,       -90,   -1.5708), ctrl::Position(  70.0179,  -69.9971,   -1.5708),   19.9821,   20.0029,   422.868); /*< T:  0.374557 [s] */
static const auto SS_F135L =    ctrl::slalom::Shape(ctrl::Position(       45,        90,   2.35619), ctrl::Position(  33.1882,   80.0001,   2.35619),   21.8117,    14.142,   353.773); /*< T:  0.464962 [s] */
static const auto SS_F135R =    ctrl::slalom::Shape(ctrl::Position(       45,       -90,  -2.35619), ctrl::Position(  33.1882,  -80.0001,  -2.35619),   21.8117,    14.142,   353.773); /*< T:  0.464962 [s] */
static const auto SS_F180L =    ctrl::slalom::Shape(ctrl::Position(        0,        90,   3.14159), ctrl::Position(        0,        90,   3.14159),        24,        24,   412.408); /*< T:  0.563056 [s] */
static const auto SS_F180R =    ctrl::slalom::Shape(ctrl::Position(        0,       -90,  -3.14159), ctrl::Position(        0,       -90,  -3.14159),        24,        24,   412.408); /*< T:  0.563056 [s] */
static const auto SS_FV90L =    ctrl::slalom::Shape(ctrl::Position(  63.6396,   63.6396,    1.5708), ctrl::Position(  48.0118,   48.0008,    1.5708),   15.6278,   15.6388,   289.946); /*< T:  0.387836 [s] */
static const auto SS_FV90R =    ctrl::slalom::Shape(ctrl::Position(  63.6396,  -63.6396,   -1.5708), ctrl::Position(  48.0118,  -48.0008,   -1.5708),   15.6278,   15.6388,   289.946); /*< T:  0.387836 [s] */
static const auto SS_FS90L =    ctrl::slalom::Shape(ctrl::Position(       45,        45,    1.5708), ctrl::Position(  44.0068,    44.001,    1.5708),  0.993221,  0.999035,    265.76); /*< T:  0.287496 [s] */
static const auto SS_FS90R =    ctrl::slalom::Shape(ctrl::Position(       45,       -45,   -1.5708), ctrl::Position(  44.0068,   -44.001,   -1.5708),  0.993221,  0.999035,    265.76); /*< T:  0.287496 [s] */
