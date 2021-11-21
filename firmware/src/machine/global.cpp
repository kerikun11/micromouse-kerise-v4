/**
 * @file global.cpp
 * @brief グローバル変数の実体を定義するC++ファイル．
 * @copyright Copyright (c) 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#include "global.h"

#include "config/io_mapping.h"
#include "config/model.h"

/* Driver */
Buzzer bz;
LED led(I2C_PORT_NUM_LED);
Motor mt(MOTOR_L_CTRL1_PIN, MOTOR_L_CTRL2_PIN, MOTOR_R_CTRL1_PIN,
         MOTOR_R_CTRL2_PIN);
Fan fan(FAN_PIN);

/* Sensor */
Button btn(BUTTON_PIN);
IMU imu;
Encoder enc(model::GearRatio *model::WheelDiameter *M_PI);
Reflector ref(PR_TX_PINS, PR_RX_PINS);
ToF tof(I2C_PORT_NUM_TOF, model::tof_dist_offset);

/* Supporter */
WallDetector wd;
SpeedController sc(model::SpeedControllerModel, model::SpeedControllerGain);
UserInterface ui;

/* Conductor */
Logger lgr;
MoveAction ma(model::TrajectoryTrackerGain);
MazeRobot mr;

/* Other */
ExternalController ec;
