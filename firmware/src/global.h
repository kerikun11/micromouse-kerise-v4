/**
   @file  global.h
   @brief グローバル変数の extern 宣言をするファイル．
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

/* Supporter */
#include "UserInterface.h"
extern UserInterface ui;
#include "SpeedController.h"
extern ctrl::SpeedController sc;
#include "WallDetector.h"
extern WallDetector wd;

/* Conductor */
#include "Logger.h"
extern Logger lgr;
#include "SearchRun.h"
extern SearchRun sr;
#include "FastRun.h"
extern FastRun fr;
#include "MazeRobot.h"
extern MazeRobot mr;
#include "ExternalController.h"
extern ExternalController ec;
