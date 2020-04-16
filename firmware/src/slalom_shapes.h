#pragma once

#include "slalom.h"

/* SS_S90L  Total Time: 0.287526 [s] */
static const auto SS_S90L  = ctrl::slalom::Shape(ctrl::Position(       45,        45,    1.5708), ctrl::Position(       44,        44,    1.5708),   1.00004,  0.999981,   265.749, 3769.91, 113.097, 9.42478);
/* SS_S90R  Total Time: 0.287526 [s] */
static const auto SS_S90R  = ctrl::slalom::Shape(ctrl::Position(       45,       -45,   -1.5708), ctrl::Position(       44,       -44,   -1.5708),   1.00004,  0.999981,   265.749, 3769.91, 113.097, 9.42478);
/* SS_F45L  Total Time: 0.257131 [s] */
static const auto SS_F45L  = ctrl::slalom::Shape(ctrl::Position(       90,        45,  0.785398), ctrl::Position(  72.4265,        30,  0.785398),   2.57355,   21.2132,   411.637, 3769.91, 113.097, 9.42478);
/* SS_F45R  Total Time: 0.257131 [s] */
static const auto SS_F45R  = ctrl::slalom::Shape(ctrl::Position(       90,       -45, -0.785398), ctrl::Position(  72.4265,       -30, -0.785398),   2.57355,   21.2132,   411.637, 3769.91, 113.097, 9.42478);
/* SS_F90L  Total Time: 0.374612 [s] */
static const auto SS_F90L  = ctrl::slalom::Shape(ctrl::Position(       90,        90,    1.5708), ctrl::Position(  69.9999,        70,    1.5708),   20.0001,        20,   422.782, 3769.91, 113.097, 9.42478);
/* SS_F90R  Total Time: 0.374612 [s] */
static const auto SS_F90R  = ctrl::slalom::Shape(ctrl::Position(       90,       -90,   -1.5708), ctrl::Position(  69.9999,       -70,   -1.5708),   20.0001,        20,   422.782, 3769.91, 113.097, 9.42478);
/* SS_F135L Total Time: 0.465157 [s] */
static const auto SS_F135L = ctrl::slalom::Shape(ctrl::Position(       45,        90,   2.35619), ctrl::Position(  33.1366,        80,   2.35619),   21.8634,   14.1421,   353.607, 3769.91, 113.097, 9.42478);
/* SS_F135R Total Time: 0.465157 [s] */
static const auto SS_F135R = ctrl::slalom::Shape(ctrl::Position(       45,       -90,  -2.35619), ctrl::Position(  33.1366,       -80,  -2.35619),   21.8634,   14.1421,   353.607, 3769.91, 113.097, 9.42478);
/* SS_F180L Total Time: 0.563108 [s] */
static const auto SS_F180L = ctrl::slalom::Shape(ctrl::Position(        0,        90,   3.14159), ctrl::Position(        0,        90,   3.14159),        24,        24,   412.225, 3769.91, 113.097, 9.42478);
/* SS_F180R Total Time: 0.563108 [s] */
static const auto SS_F180R = ctrl::slalom::Shape(ctrl::Position(        0,       -90,  -3.14159), ctrl::Position(        0,       -90,  -3.14159),        24,        24,   412.225, 3769.91, 113.097, 9.42478);
/* SS_FV90L Total Time: 0.387894 [s] */
static const auto SS_FV90L = ctrl::slalom::Shape(ctrl::Position(  63.6396,   63.6396,    1.5708), ctrl::Position(  47.9999,        48,    1.5708),   15.6397,   15.6396,   289.908, 3769.91, 113.097, 9.42478);
/* SS_FV90R Total Time: 0.387894 [s] */
static const auto SS_FV90R = ctrl::slalom::Shape(ctrl::Position(  63.6396,  -63.6396,   -1.5708), ctrl::Position(  47.9999,       -48,   -1.5708),   15.6397,   15.6396,   289.908, 3769.91, 113.097, 9.42478);
/* SS_FS90L Total Time: 0.287526 [s] */
static const auto SS_FS90L = ctrl::slalom::Shape(ctrl::Position(       45,        45,    1.5708), ctrl::Position(       44,        44,    1.5708),   1.00004,  0.999981,   265.749, 3769.91, 113.097, 9.42478);
/* SS_FS90R Total Time: 0.287526 [s] */
static const auto SS_FS90R = ctrl::slalom::Shape(ctrl::Position(       45,       -45,   -1.5708), ctrl::Position(       44,       -44,   -1.5708),   1.00004,  0.999981,   265.749, 3769.91, 113.097, 9.42478);
