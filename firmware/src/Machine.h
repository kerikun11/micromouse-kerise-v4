#pragma once

#include "app_log.h"

#include "global.h"

#include "config.h"

#include <Arduino.h>
#include <SPIFFS.h>
#include <WiFi.h>

class Machine {
public:
  static bool init() {
    /* pullup all the pins of the SPI-CS so that the bus is not blocked  */
    for (auto p : CONFIG_SPI_CS_PINS)
      pinMode(p, INPUT_PULLUP);
    /* Buzzer */
    bz.begin();
    /* I2C */
    if (!I2C::install(I2C_PORT_NUM, I2C_SDA_PIN, I2C_SCL_PIN))
      bz.play(Buzzer::ERROR);
    if (!led.begin())
      bz.play(Buzzer::ERROR);
    ui.batteryCheck();
    bz.play(Buzzer::BOOT);

    if (!SPIFFS.begin(true))
      bz.play(Buzzer::ERROR);
    if (!SPI::busInit(CONFIG_SPI_HOST, CONFIG_SPI_SCLK_PIN, CONFIG_SPI_MISO_PIN,
                      CONFIG_SPI_MOSI_PIN, CONFIG_SPI_DMA_CHAIN))
      bz.play(Buzzer::ERROR);
    if (!imu.begin(ICM20602_SPI_HOST, ICM20602_CS_PINS))
      bz.play(Buzzer::ERROR);
    if (!enc.begin(AS5048A_SPI_HOST, AS5048A_CS_PIN))
      bz.play(Buzzer::ERROR);
    if (!ref.begin())
      bz.play(Buzzer::ERROR);
    if (!tof.begin())
      bz.play(Buzzer::ERROR);
    if (!wd.begin())
      bz.play(Buzzer::ERROR);

    if (!ec.begin())
      bz.play(Buzzer::ERROR);
    return true;
  }
  static void driveNormally() {
    if (mr.isComplete() && !mr.calcShortestDirs(true)) {
      bz.play(Buzzer::ERROR);
      mr.resetLastWall(3);
      return;
    }
    if (mr.isComplete())
      bz.play(Buzzer::MAZE_RESTORE);
    else
      bz.play(Buzzer::MAZE_BACKUP);
    int mode = ui.waitForSelect(4);
    bool pi_enabled = true;
    bool forceSearch = false;
    bool posIdAtStart = false;
    switch (mode) {
    case 0:
      break;
    case 1:
      pi_enabled = false;
      break;
    case 2:
      posIdAtStart = true;
      break;
    case 3:
      forceSearch = true;
      break;
    }
    if (!ui.waitForCover())
      return;
    led = 9;
    delay(3000);
    mr.start(forceSearch, posIdAtStart);
    while (mr.isRunning()) {
      if (mt.isEmergency()) {
        bz.play(Buzzer::EMERGENCY);
        mr.terminate();
        fan.free();
        delay(1000);
        mt.emergencyRelease();
        tof.enable(); /*< EmergencyStopのタイミング次第でdisabledの場合がある */
        if (!pi_enabled)
          break;
        mr.start(false, true); /*< Position Identification Run */
      }
      delay(100);
    }
    mr.terminate();
  }
  static void selectParamPreset() {
    int value;

    for (int i = 0; i < 1; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    fr.runParameter.curve_gain = fr.runParameter.getCurveGains(value);

    for (int i = 0; i < 2; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    fr.runParameter.max_speed = fr.runParameter.getMaxSpeeds(value);

    for (int i = 0; i < 3; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    fr.runParameter.accel = fr.runParameter.getAccels(value);

    bz.play(Buzzer::SUCCESSFUL);
  }
  static void selectParamManually() {
    int value;

    for (int i = 0; i < 1; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    fr.runParameter.curve_gain *= std::pow(1.1f, value);

    for (int i = 0; i < 2; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    fr.runParameter.max_speed *= std::pow(1.1f, value);

    for (int i = 0; i < 3; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    fr.runParameter.accel *= std::pow(1.21f, value);
    bz.play(Buzzer::SUCCESSFUL);
  }
  // static void selectParamManually() {
  //   int value;
  //   for (int i = 0; i < 1; i++)
  //     bz.play(Buzzer::SHORT);
  //   value = ui.waitForSelect(16);
  //   if (value < 0)
  //     return;
  //   const float curve_gain = 0.1f * value;
  //   for (int i = 0; i < 2; i++)
  //     bz.play(Buzzer::SHORT);
  //   value = ui.waitForSelect(16);
  //   if (value < 0)
  //     return;
  //   const float v_max = 300.0f * value;
  //   for (int i = 0; i < 3; i++)
  //     bz.play(Buzzer::SHORT);
  //   value = ui.waitForSelect(16);
  //   if (value < 0)
  //     return;
  //   const float accel = 600.0f * value;
  //   fr.runParameter = FastRun::RunParameter(curve_gain, v_max, accel);
  //   bz.play(Buzzer::SUCCESSFUL);
  // }
  static void selectFanGain() {
    fan.drive(0.5f);
    delay(100);
    fan.drive(0);
    int value = ui.waitForSelect(11);
    if (value < 0)
      return;
    fr.fanDuty = 0.1f * value;
    fan.drive(fr.fanDuty);
    ui.waitForSelect(1);
    fan.drive(0);
    bz.play(Buzzer::SUCCESSFUL);
  }
  static void partyStunt() {
    if (!ui.waitForCover())
      return;
    led = 6;
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    led = 9;
    sc.enable();
    sc.set_target(0, 0);
    ui.waitForCover();
    sc.disable();
  }
  static void wallCalibration() {
    int mode = ui.waitForSelect(3);
    switch (mode) {
    /* 前壁補正データの保存 */
    case 0:
      led = 15;
      if (!ui.waitForCover())
        return;
      if (wd.backup()) {
        bz.play(Buzzer::SUCCESSFUL);
      } else {
        bz.play(Buzzer::ERROR);
      }
      break;
    /* 横壁キャリブレーション */
    case 1:
      led = 9;
      if (!ui.waitForCover())
        return;
      delay(1000);
      bz.play(Buzzer::CONFIRM);
      wd.calibrationSide();
      bz.play(Buzzer::CANCEL);
      break;
    /* 前壁キャリブレーション */
    case 2:
      led = 6;
      if (!ui.waitForCover(true))
        return;
      delay(1000);
      bz.play(Buzzer::CONFIRM);
      wd.calibrationFront();
      bz.play(Buzzer::CANCEL);
      break;
    }
  }
  static void setGoalPositions() {
    for (int i = 0; i < 2; i++)
      bz.play(Buzzer::SHORT);
    int value = ui.waitForSelect(5);
    if (value < 0)
      return;
    switch (value) {
    case 0: {
      int x = ui.waitForSelect(16);
      int y = ui.waitForSelect(16);
      mr.set_goal({Vector(x, y)});
      break;
    }
    case 1:
      mr.set_goal({Vector(1, 0)});
      break;
    case 2:
      mr.set_goal({Vector(9, 9), Vector(10, 10), Vector(10, 9), Vector(9, 10)});
      break;
    case 3:
      mr.set_goal({Vector(3, 3)});
      break;
    case 4:
      mr.set_goal({Vector(15, 15)});
      break;
    }
    bz.play(Buzzer::SUCCESSFUL);
  }
  static void petitcon() {
    if (!ui.waitForCover())
      return;
    delay(500);
    // fr.set_path("sssssssrlrlrlrlrlrlssssslrlrlrlrlrlrsssssrlrlrlrlrlrssssssssrl"
    //             "rlrlrlrlrssssssssssssssssssslrlrlrlrlrlsssssssslrlrlrlrlrlssss"
    //             "slrlrlrlrlrlrsssssrlrlrlrlrlrlssssss");
    std::string path;
    path += "s";
    for (int j = 0; j < 4; ++j) {
      for (int i = 0; i < 28; ++i)
        path += "s";
      path += "r";
      for (int i = 0; i < 12; ++i)
        path += "s";
      path += "r";
    }
    path += "s";
    fr.set_path(path);
    fr.start();
    while (fr.isRunning()) {
      delay(100);
      if (mt.isEmergency()) {
        bz.play(Buzzer::EMERGENCY);
        fr.terminate();
        fan.free();
        delay(100);
        mt.emergencyRelease();
        break;
      }
    }
  }
  static void sysid() {
    int dir = ui.waitForSelect(2);
    int gain = ui.waitForSelect();
    if (gain < 0)
      return;
    if (!ui.waitForCover())
      return;
    delay(1000);
    lgr.clear();
    auto printLog = []() {
      lgr.push({
          enc.position(0),
          enc.position(1),
          imu.gyro.z,
          imu.accel.y,
          imu.angular_accel,
          ui.getBatteryVoltage(),
      });
    };
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    fan.drive(0.5f);
    delay(500);
    portTickType xLastWakeTime = xTaskGetTickCount();
    if (dir == 1)
      mt.drive(-gain * 0.05, gain * 0.05); //< 回転
    else
      mt.drive(gain * 0.1, gain * 0.1); //< 並進
    for (int i = 0; i < 2000; i++) {
      printLog();
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    }
    fan.drive(0);
    mt.drive(0, 0);
    delay(500);
    mt.free();
  }
  static void accel_test() {
    if (!ui.waitForCover())
      return;
    delay(500);
    lgr.clear();
    auto printLog = []() {
      auto &bd = sc.fbc.getBreakdown();
      lgr.push({
          sc.ref_v.tra,
          sc.est_v.tra,
          sc.ref_a.tra,
          sc.est_a.tra,
          bd.ff.tra,
          bd.fb.tra,
          bd.fbp.tra,
          bd.fbi.tra,
          bd.u.tra,
          sc.ref_v.rot,
          sc.est_v.rot,
          sc.ref_a.rot,
          sc.est_a.rot,
          bd.ff.rot,
          bd.fb.rot,
          bd.fbp.rot,
          bd.fbi.rot,
          bd.u.rot,
      });
    };
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    fan.drive(0.4);
    delay(500);
    sc.enable();
    const float jerk = 500000;
    const float accel = 6000;
    const float v_max = 1200;
    AccelDesigner ad(jerk, accel, 0, v_max, 0, 90 * 8);
    portTickType xLastWakeTime = xTaskGetTickCount();
    for (float t = 0; t < ad.t_end() + 0.1f; t += 0.001f) {
      sc.set_target(ad.v(t), 0, ad.a(t), 0);
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      if ((int)(t * 1000) % 2 == 0)
        printLog();
      if (mt.isEmergency()) {
        bz.play(Buzzer::EMERGENCY);
        break;
      }
    }
    sc.set_target(0, 0);
    delay(200);
    bz.play(Buzzer::CANCEL);
    sc.disable();
    fan.drive(0);
  }
  static void SearchRun_test() {
    if (!ui.waitForCover())
      return;
    delay(500);
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    sr.set_action(SearchRun::START_STEP);
    sr.set_action(SearchRun::TURN_RIGHT_90);
    sr.set_action(SearchRun::TURN_LEFT_90);
    sr.set_action(SearchRun::TURN_LEFT_90);
    sr.set_action(SearchRun::TURN_RIGHT_90);
    sr.set_action(SearchRun::TURN_RIGHT_90);
    sr.set_action(SearchRun::GO_STRAIGHT);
    sr.set_action(SearchRun::STOP);
    sr.enable();
    while (sr.isRunning()) {
      if (mt.isEmergency()) {
        bz.play(Buzzer::EMERGENCY);
        sr.disable();
        delay(1000);
        mt.emergencyRelease();
        break;
      }
      delay(100);
    }
  }
  static void position_recovery(const bool pi_enabled = false) {
    while (1) {
      if (!ui.waitForCover(true))
        return;
      led = 0;
      if (!sr.positionRecovery())
        bz.play(Buzzer::ERROR);
      led = 15;
    }
  }
};
