/**
 * @file global.h
 * @brief グローバル変数の extern 宣言をするファイル．
 */
#pragma once

/* Driver */
#include "hardware/buzzer.h"
extern Buzzer bz;
#include "hardware/led.h"
extern LED led;
#include "hardware/motor.h"
extern Motor mt;
#include "hardware/fan.h"
extern Fan fan;

/* Sensor */
#include "hardware/button.h"
extern Button btn;
#include "hardware/imu.h"
extern IMU imu;
#include "hardware/encoder.h"
extern Encoder enc;
#include "hardware/reflector.h"
extern Reflector ref;
#include "hardware/tof.h"
extern ToF tof;

/* Logger */
#include "logger.h"
extern Logger lgr;

/* Supporter */
#include "wall_detector.h"
extern WallDetector wd;
#include "speed_controller.h"
extern SpeedController sc;
#include "user_interface.h"
extern UserInterface ui;

/* Conductor */
#include "move_action.h"
extern MoveAction ma;
#include "maze_robot.h"
extern MazeRobot mr;