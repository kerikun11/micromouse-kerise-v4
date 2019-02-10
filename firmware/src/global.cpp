/**
   @file  global.cpp
   @brief グローバル変数の実体を定義するC++ファイル．
*/
#include "global.h"

#include "config/config.h"

/* Driver */
Buzzer bz(BUZZER_PIN, LEDC_CH_BUZZER);
LED led(I2C_PORT_NUM_LED);
Motor mt(MOTOR_L_CTRL1_PIN, MOTOR_L_CTRL2_PIN, MOTOR_R_CTRL1_PIN,
         MOTOR_R_CTRL2_PIN);
Fan fan(FAN_PIN);

/* Sensor */
Button btn(BUTTON_PIN);
IMU imu;
Encoder enc(MACHINE_GEAR_RATIO, MACHINE_WHEEL_DIAMETER);
Reflector ref(PR_TX_PINS, PR_RX_PINS);
ToF tof(I2C_PORT_NUM_TOF);

/* Supporter */
UserInterface ui;
SpeedController sc;
WallDetector wd;

/* Conductor */
Logger lgr;
SearchRun sr;
FastRun fr;
MazeRobot mr;

/* Other */
ExternalController ec;