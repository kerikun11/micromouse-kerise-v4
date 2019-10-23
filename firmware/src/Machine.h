#pragma once

#include "app_log.h"

#include "global.h"

#include "config/io_mapping.h"

#include <Arduino.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <json11.hpp>

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
#if KERISE_SELECT == 4 || KERISE_SELECT == 3
    if (!imu.begin(ICM20602_SPI_HOST, ICM20602_CS_PINS))
      bz.play(Buzzer::ERROR);
    if (!enc.begin(AS5048A_SPI_HOST, AS5048A_CS_PIN))
      bz.play(Buzzer::ERROR);
#elif KERISE_SELECT == 5
    if (!imu.begin(ICM20602_SPI_HOST, ICM20602_CS_PIN))
      bz.play(Buzzer::ERROR);
    if (!enc.begin(MA730_SPI_HOST, MA730_CS_PINS))
      bz.play(Buzzer::ERROR);
#endif
    if (!ref.begin())
      bz.play(Buzzer::ERROR);
    if (!tof.begin())
      bz.play(Buzzer::ERROR);
    if (!wd.begin())
      bz.play(Buzzer::ERROR);

    // if (!ec.begin())
    // bz.play(Buzzer::ERROR);
    return true;
  }
  static void restore() {
    if (!mr.restore())
      bz.play(Buzzer::ERROR);
    else
      bz.play(Buzzer::MAZE_RESTORE);
  }
  static void reset() {
    if (!ui.waitForCover())
      return;
    bz.play(Buzzer::MAZE_BACKUP);
    mr.reset();
  }
  static void driveAutomatically() {
    restore();
    if (!ui.waitForPickup())
      return;
    mr.autoRun(false, true);
  }
  static void driveNormally() {
    /* 異常検出 */
    if (mr.isComplete() && !mr.calcShortestDirections(true)) {
      bz.play(Buzzer::ERROR);
      mr.resetLastWalls(3);
      return;
    }
    /* 探索状態のお知らせ */
    if (mr.getMaze().getWallLogs().empty()) //< 完全に未探索状態
      bz.play(Buzzer::MAZE_BACKUP);
    else if (mr.isComplete()) //< 完全に探索終了
      bz.play(Buzzer::SUCCESSFUL);
    else if (mr.calcShortestDirections(true)) //< 探索中だが経路はある
      bz.play(Buzzer::CONFIRM);
    /* 走行オプション */
    int mode = ui.waitForSelect(2);
    if (mode < 0)
      return;
    bool forceSearch = false;
    switch (mode) {
    case 0:
      //< デフォルト
      break;
    case 1:
      forceSearch = true;
      break;
    }
    if (!ui.waitForCover())
      return;
    led = 9;
    // delay(3000); //< 動画用 delay
    mr.autoRun(forceSearch, false);
  }
  static void selectParamPreset() {
    sr.rp_fast = SearchRun::RunParameter();
    bz.play(Buzzer::SUCCESSFUL);
  }
  static void selectRunConfig() {
    int value = ui.waitForSelect(16);
    if (value < 0)
      return;
    sr.rp_search.diag_enabled = value & 0x01;
    sr.rp_fast.diag_enabled = value & 0x02;
    sr.rp_search.front_wall_fix_enabled = value & 0x04;
    sr.rp_fast.front_wall_fix_enabled = value & 0x04;
    sr.rp_search.wall_avoid_enabled = value & 0x08;
    sr.rp_fast.wall_avoid_enabled = value & 0x08;
    bz.play(Buzzer::SUCCESSFUL);
  }
  static void selectParamManually() {
    int value;
    /* ターン速度 */
    for (int i = 0; i < 1; i++)
      bz.play(Buzzer::SHORT7);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    sr.rp_fast.curve_gain *= std::pow(sr.rp_fast.cg_gain, value);
    /* 最大速度 */
    for (int i = 0; i < 2; i++)
      bz.play(Buzzer::SHORT7);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    sr.rp_fast.max_speed *= std::pow(sr.rp_fast.ms_gain, value);
    /* 加速度 */
    for (int i = 0; i < 3; i++)
      bz.play(Buzzer::SHORT7);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    sr.rp_fast.accel *= std::pow(sr.rp_fast.ms_gain, value);
    /* 成功 */
    bz.play(Buzzer::SUCCESSFUL);
  }
  static void selectFanGain() {
    fan.drive(0.5f);
    delay(100);
    fan.drive(0);
    int value = ui.waitForSelect(11);
    if (value < 0)
      return;
    sr.rp_fast.fan_duty = 0.1f * value;
    fan.drive(sr.rp_fast.fan_duty);
    // mt.drive(sr.fan_duty, sr.fan_duty);
    ui.waitForSelect(1);
    fan.drive(0);
    mt.drive(0, 0);
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
      bz.play(Buzzer::SHORT7);
    int value = ui.waitForSelect(5);
    if (value < 0)
      return;
    switch (value) {
    case 0: {
      int x = ui.waitForSelect(16);
      int y = ui.waitForSelect(16);
      mr.setGoals({MazeLib::Position(x, y)});
      break;
    }
    case 1:
      mr.setGoals({MazeLib::Position(1, 0)});
      break;
    case 2:
      mr.setGoals({MazeLib::Position(6, 9), MazeLib::Position(6, 10),
                   MazeLib::Position(7, 9), MazeLib::Position(7, 10)});
      break;
    case 3:
      mr.setGoals({MazeLib::Position(3, 3), MazeLib::Position(3, 4),
                   MazeLib::Position(4, 3), MazeLib::Position(4, 4)});
      break;
    case 4:
      mr.setGoals({MazeLib::Position(15, 15)});
      break;
    }
    bz.play(Buzzer::SUCCESSFUL);
  }
  static void petitcon() {
    if (!ui.waitForCover())
      return;
    delay(500);
    // sr.set_path("sssssssrlrlrlrlrlrlssssslrlrlrlrlrlrsssssrlrlrlrlrlrssssssssrl"
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
    sr.set_path(path);
    sr.enable();
    while (sr.isRunning()) {
      delay(100);
      if (mt.isEmergency()) {
        bz.play(Buzzer::EMERGENCY);
        sr.disable();
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
    int dir = ui.waitForSelect(2);
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
    AccelDesigner ad;
    if (dir == 0) {
      const float jerk = 240000;
      const float accel = 9000;
      const float v_max = 1200;
      const float dist = 90 * 6;
      ad.reset(jerk, accel, 0, v_max, 0, dist);
    } else {
      const float jerk = 2400 * M_PI;
      const float accel = 48 * M_PI;
      const float v_max = 6 * M_PI;
      const float dist = 2 * M_PI;
      ad.reset(jerk, accel, 0, v_max, 0, dist);
    }
    portTickType xLastWakeTime = xTaskGetTickCount();
    for (float t = 0; t < ad.t_end() + 0.1f; t += 0.001f) {
      if (dir == 0)
        sc.set_target(ad.v(t), 0, ad.a(t), 0);
      else
        sc.set_target(0, ad.v(t), 0, ad.a(t));
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
  static void slalom_test() {
    if (!ui.waitForCover())
      return;
    delay(500);
    lgr.clear();
    auto printLog = [](const auto ref_q, const auto est_q) {
      auto &bd = sc.fbc.getBreakdown();
      lgr.push({
          sc.ref_v.tra, sc.est_v.tra, sc.ref_a.tra, sc.est_a.tra, bd.ff.tra,
          bd.fb.tra,    bd.fbp.tra,   bd.fbi.tra,   bd.u.tra,     sc.ref_v.rot,
          sc.est_v.rot, sc.ref_a.rot, sc.est_a.rot, bd.ff.rot,    bd.fb.rot,
          bd.fbp.rot,   bd.fbi.rot,   bd.u.rot,     ref_q.x,      est_q.x,
          ref_q.y,      est_q.y,      ref_q.th,     est_q.th,
      });
    };
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    const float velocity = 450;
    const float Ts = 0.001f;
    const float j_max = 240000;
    const float a_max = 9000;
    const float v_max = velocity;
    const float dist = 1 * 90;
    ctrl::TrajectoryTracker tt{model::TrajectoryTrackerGain};
    const auto &shape = SS_FL90;
    ctrl::Position offset;
    /* start */
    sc.enable();
    portTickType xLastWakeTime = xTaskGetTickCount();
    tt.reset(0);
    /* accel */
    ctrl::straight::Trajectory ref;
    ref.reset(j_max, a_max, 0, v_max, velocity, dist + shape.straight_prev);
    for (float t = 0; t < ref.t_end(); t += Ts) {
      ctrl::State ref_s;
      ref.update(ref_s, t);
      auto est_q = sc.position;
      auto ref = tt.update(est_q, sc.est_v, sc.est_a, ref_s);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      printLog(ref_s.q.homogeneous(offset), est_q.homogeneous(offset));
    }
    sc.position.x -= ref.x_end();
    offset += ctrl::Position(ref.x_end(), 0, 0).rotate(offset.th);
    /* slalom */
    slalom::Trajectory st(shape);
    st.reset(velocity);
    ctrl::State ref_s;
    for (float t = 0; t < st.t_end(); t += Ts) {
      st.update(ref_s, t, Ts);
      auto est_q = sc.position;
      auto ref = tt.update(est_q, sc.est_v, sc.est_a, ref_s);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      printLog(ref_s.q.homogeneous(offset), est_q.homogeneous(offset));
    }
    const auto &net = st.get_net_curve();
    sc.position = (sc.position - net).rotate(-net.th);
    offset += net.rotate(offset.th);
    /* decel */
    ref.reset(j_max, a_max, sc.ref_v.tra, v_max, 0, dist + shape.straight_post);
    for (float t = 0; t < ref.t_end(); t += Ts) {
      ctrl::State ref_s;
      ref.update(ref_s, t);
      auto est_q = sc.position;
      auto ref = tt.update(est_q, sc.est_v, sc.est_a, ref_s);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      printLog(ref_s.q.homogeneous(offset), est_q.homogeneous(offset));
    }
    sc.position.x -= ref.x_end();
    offset += ctrl::Position(ref.x_end(), 0, 0).rotate(offset.th);
    /* end */
    sc.set_target(0, 0);
    delay(200);
    bz.play(Buzzer::CANCEL);
    sc.disable();
  }
  static void SearchRun_test() {
    if (!ui.waitForCover())
      return;
    delay(500);
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    // sr.set_action(SearchRun::START_STEP);
    // sr.set_action(SearchRun::TURN_RIGHT_90);
    // sr.set_action(SearchRun::TURN_LEFT_90);
    // sr.set_action(SearchRun::TURN_LEFT_90);
    // sr.set_action(SearchRun::TURN_RIGHT_90);
    // sr.set_action(SearchRun::TURN_RIGHT_90);
    // sr.set_action(SearchRun::GO_STRAIGHT);
    // sr.set_action(SearchRun::STOP);
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
      delay(1000);
      if (!sr.positionRecovery())
        bz.play(Buzzer::ERROR);
      led = 15;
    }
  }
};
