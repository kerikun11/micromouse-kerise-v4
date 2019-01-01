/**
   @file  global.cpp
   @brief グローバル変数の実体を定義するC++ファイル．
*/
#include "global.h"
#include "config/config.h"

/* Driver */
#include "buzzer.h"
Buzzer bz(BUZZER_PIN, LEDC_CH_BUZZER);
#include "led.h"
LED led(I2C_PORT_NUM_LED);
#include "motor.h"
Motor mt(MOTOR_L_CTRL1_PIN, MOTOR_L_CTRL2_PIN, MOTOR_R_CTRL1_PIN,
         MOTOR_R_CTRL2_PIN, LEDC_CH_MOTOR_L_CTRL1, LEDC_CH_MOTOR_L_CTRL2,
         LEDC_CH_MOTOR_R_CTRL1, LEDC_CH_MOTOR_R_CTRL2);
#include "fan.h"
Fan fan(FAN_PIN, LEDC_CH_FAN);

/* Sensor */
#include "button.h"
Button btn(BUTTON_PIN);
#include "imu.h"
IMU imu;
#include "encoder.h"
Encoder enc(MACHINE_GEAR_RATIO, MACHINE_WHEEL_DIAMETER);
#include "reflector.h"
Reflector ref(PR_TX_PINS, PR_RX_PINS);
#include "tof.h"
ToF tof(I2C_PORT_NUM_TOF);

/* Supporter */
#include "UserInterface.h"
UserInterface ui;
#include "SpeedController.h"
SpeedController sc;
#include "WallDetector.h"
WallDetector wd;
#include "Emergency.h"
Emergency em;

/* Conductor */
#include "Logger.h"
Logger lgr;
#include "SearchRun.h"
SearchRun sr;
#include "FastRun.h"
FastRun fr;
#include "MazeRobot.h"
MazeRobot mr;
