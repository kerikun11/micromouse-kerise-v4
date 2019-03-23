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
    if (mr.isComplete() && !mr.calcShortestDirs()) {
      bz.play(Buzzer::ERROR);
      mr.resetLastWall(4);
      return;
    }
    if (mr.isComplete())
      bz.play(Buzzer::SUCCESSFUL);
    if (!ui.waitForCover())
      return;
    led = 9;
    mr.start();
    while (mr.isRunning()) {
      if (mt.isEmergency()) {
        bz.play(Buzzer::EMERGENCY);
        mr.terminate();
        fan.free();
        delay(1000);
        mt.emergencyRelease();
        break; //< for debug
        mr.start(false, true);
      }
      delay(100);
    }
    mr.terminate();
  }
  static void selectParamPreset() {
    int preset = ui.waitForSelect(16);
    if (preset < 0)
      return;
    float gains[4] = {0.5, 0.6, 0.7, 0.8};
    float vmaxs[4] = {600, 600, 720, 720};
    float accels[4] = {1200, 2400, 3600, 7200};
    fr.runParameter =
        FastRun::RunParameter(gains[(preset >> 2) & 3], vmaxs[preset & 3],
                              accels[preset & 3], accels[preset & 3] / 2);
    bz.play(Buzzer::SUCCESSFUL);
  }
  static void selectParamManually() {
    int value;
    for (int i = 0; i < 1; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    const float curve_gain = 0.1f * value;
    for (int i = 0; i < 2; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    const float v_max = 300.0f * value;
    for (int i = 0; i < 3; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    const float accel = 600.0f * value;
    fr.runParameter = FastRun::RunParameter(curve_gain, v_max, accel, accel);
    bz.play(Buzzer::SUCCESSFUL);
  }
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
    bz.play(Buzzer::CONFIRM);
    imu.calibration();
    bz.play(Buzzer::CANCEL);
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
      mr.set_goal({Vector(8, 8)});
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
    // path += "srrlrllsllrlrr";
    // path += "rllrrssrrllrrssr";
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
    bz.play(Buzzer::SELECT);
    imu.calibration();
    bz.play(Buzzer::CANCEL);
    fan.drive(0.5f);
    delay(500);
    portTickType xLastWakeTime = xTaskGetTickCount();
    mt.drive(-gain * 0.05, gain * 0.05); //< 回転
    // mt.drive(gain * 0.1, gain * 0.1);    //< 並進
    for (int i = 0; i < 2000; i++) {
      printLog();
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    }
    fan.drive(0);
    mt.drive(0, 0);
    delay(500);
    mt.free();
  }
};
