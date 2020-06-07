#pragma once

#include "app_log.h"

#include "global.h"

#include "config/io_mapping.h"
#include "peripheral/spiffs.h"

#include <WiFi.h>

class Machine {
public:
  static bool init() {
    /* pullup all the pins of the SPI-CS so that the bus is not blocked  */
    for (auto p : CONFIG_SPI_CS_PINS)
      pinMode(p, INPUT_PULLUP);
    pinMode(RX, INPUT_PULLUP);
    /* Buzzer */
    bz.init(BUZZER_PIN, LEDC_CH_BUZZER);
    /* I2C */
    if (!peripheral::I2C::install(I2C_PORT_NUM, I2C_SDA_PIN, I2C_SCL_PIN))
      bz.play(Buzzer::ERROR);
    if (!led.init())
      bz.play(Buzzer::ERROR);
    ui.batteryCheck();
    bz.play(Buzzer::BOOT);

    if (!peripheral::SPIFFS::init())
      bz.play(Buzzer::ERROR);
    if (!peripheral::SPI::install(CONFIG_SPI_HOST, CONFIG_SPI_SCLK_PIN,
                                  CONFIG_SPI_MISO_PIN, CONFIG_SPI_MOSI_PIN,
                                  CONFIG_SPI_DMA_CHAIN))
      bz.play(Buzzer::ERROR);
    if (!imu.init(ICM20602_SPI_HOST, ICM20602_CS_PINS))
      bz.play(Buzzer::ERROR);
#if KERISE_SELECT == 4 || KERISE_SELECT == 3
    if (!enc.init(AS5048A_SPI_HOST, AS5048A_CS_PIN))
      bz.play(Buzzer::ERROR);
#elif KERISE_SELECT == 5
    if (!enc.init(MA730_SPI_HOST, MA730_CS_PINS))
      bz.play(Buzzer::ERROR);
#endif
    if (!ref.init())
      bz.play(Buzzer::ERROR);
    if (!tof.init())
      bz.play(Buzzer::ERROR);
    if (!wd.init())
      bz.play(Buzzer::ERROR);
    if (!sc.init())
      bz.play(Buzzer::ERROR);

    // if (!ec.init())
    //   bz.play(Buzzer::ERROR);
    return true;
  }
  static void restore() {
    if (!mr.restore())
      bz.play(Buzzer::ERROR);
    else
      bz.play(Buzzer::MAZE_RESTORE);
    /* ゴールが封印されていないか一応確認 */
    if (!mr.isSolvable())
      bz.play(Buzzer::ERROR);
  }
  static void reset() {
    if (!ui.waitForCover())
      return;
    bz.play(Buzzer::MAZE_BACKUP);
    mr.reset();
  }
  static void driveAutomatically() {
    restore();
    if (ui.waitForPickup())
      return;
    mr.autoRun();
  }
  static void driveNormally() {
    /* 探索状態のお知らせ */
    if (mr.getMaze().getWallRecords().empty()) //< 完全に未探索状態
      bz.play(Buzzer::CONFIRM);
    else if (mr.isComplete()) //< 完全に探索終了
      bz.play(Buzzer::SUCCESSFUL);
    else if (mr.calcShortestDirections(true)) //< 探索中だが経路はある
      bz.play(Buzzer::MAZE_RESTORE);
    else //< 行きの探索中
      bz.play(Buzzer::MAZE_BACKUP);
    /* 異常検出 */
    if (!mr.isSolvable()) {
      bz.play(Buzzer::ERROR);
      mr.resetLastWalls(6);
      return;
    }
    /* 走行オプション */
    int mode = ui.waitForSelect(2);
    if (mode < 0)
      return;
    bool forceSearch = false;
    switch (mode) {
    case 0: /*< デフォルト */
      break;
    case 1: /*< 強制探索モード */
      forceSearch = true;
      break;
    }
    if (!ui.waitForCover())
      return;
    led = 9;
    // delay(5000); //< 動画用 delay
    mr.autoRun(forceSearch);
  }
  static void selectParamPreset() {
    for (int i = 0; i < 1; i++)
      bz.play(Buzzer::SHORT7);
    int value = ui.waitForSelect(16);
    if (value < 0)
      return;
    ma.rp_fast = MoveAction::RunParameter();
    if (value <= 7)
      ma.rp_fast.up(value);
    else
      ma.rp_fast.down(16 - value);
    bz.play(Buzzer::SUCCESSFUL);
  }
  static void selectRunConfig() {
    int mode = ui.waitForSelect(16);
    if (mode < 0)
      return;
    int value = ui.waitForSelect(4);
    if (value < 0)
      return;
    switch (mode) {
    case 0: /* 斜め走行 */
      ma.rp_search.diag_enabled = value & 0x01;
      ma.rp_fast.diag_enabled = value & 0x02;
      break;
    case 1: /* 未知区間加速 */
      ma.rp_search.unknown_accel_enabled = value & 0x01;
      ma.rp_fast.unknown_accel_enabled = value & 0x02;
      break;
    case 2: /* 前壁修正 */
      ma.rp_search.front_wall_fix_enabled = value & 1;
      ma.rp_fast.front_wall_fix_enabled = value & 2;
      break;
    case 3: /* 横壁補正 */
      ma.rp_search.wall_avoid_enabled = value & 1;
      ma.rp_fast.wall_avoid_enabled = value & 2;
      break;
    case 4: /* 横壁姿勢補正 */
      ma.rp_search.wall_theta_fix_enabled = value & 1;
      ma.rp_fast.wall_theta_fix_enabled = value & 2;
      break;
    case 5: /* 壁切れ */
      ma.rp_search.wall_cut_enabled = value & 1;
      ma.rp_fast.wall_cut_enabled = value & 2;
      break;
    }
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
    for (auto &vs : ma.rp_fast.v_slalom)
      vs *= std::pow(ma.rp_fast.vs_factor, float(value));
    /* 最大速度 */
    for (int i = 0; i < 2; i++)
      bz.play(Buzzer::SHORT7);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    ma.rp_fast.v_max *= std::pow(ma.rp_fast.vm_factor, float(value));
    /* 加速度 */
    for (int i = 0; i < 3; i++)
      bz.play(Buzzer::SHORT7);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    ma.rp_fast.a_max *= std::pow(ma.rp_fast.vm_factor, float(value));
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
    ma.rp_fast.fan_duty = 0.1f * value;
    fan.drive(ma.rp_fast.fan_duty);
    // mt.drive(ma.fan_duty, ma.fan_duty);
    ui.waitForSelect(1);
    fan.drive(0);
    mt.drive(0, 0);
    bz.play(Buzzer::SUCCESSFUL);
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
      wd.calibration_side();
      bz.play(Buzzer::CANCEL);
      break;
    /* 前壁キャリブレーション */
    case 2:
      led = 6;
      if (!ui.waitForCover(true))
        return;
      delay(1000);
      bz.play(Buzzer::CONFIRM);
      wd.calibration_front();
      bz.play(Buzzer::CANCEL);
      break;
    }
  }
  static void setGoalPositions() {
    for (int i = 0; i < 2; i++)
      bz.play(Buzzer::SHORT7);
    int value = ui.waitForSelect(6);
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
      mr.setGoals({
          MazeLib::Position(17, 13),
          MazeLib::Position(18, 13),
          MazeLib::Position(19, 13),
          MazeLib::Position(17, 14),
          MazeLib::Position(18, 14),
          MazeLib::Position(19, 14),
          MazeLib::Position(17, 15),
          MazeLib::Position(18, 15),
          MazeLib::Position(19, 15),
      });
      break;
    case 3:
      mr.setGoals({MazeLib::Position(3, 3), MazeLib::Position(3, 4),
                   MazeLib::Position(4, 3), MazeLib::Position(4, 4)});
      break;
    case 4:
      mr.setGoals({MazeLib::Position(8, 8)});
      break;
    case 5:
      mr.setGoals({MazeLib::Position(15, 15)});
      break;
    }
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
    while (!mt.is_emergency())
      vTaskDelay(pdMS_TO_TICKS(1));
    mt.drive(0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    sc.disable();
    mt.emergency_release();
    bz.play(Buzzer::CANCEL);
  }
  static void petitcon() {
    if (!ui.waitForCover())
      return;
    delay(500);
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
    ma.set_fast_path(path);
    ma.enable(MoveAction::TaskActionFastRun);
    ma.waitForEndAction();
    ma.disable();
    ma.emergency_release();
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
    const auto printLog = [&]() {
      lgr.push({
          enc.get_position(0),
          enc.get_position(1),
          imu.gyro.z,
          imu.accel.y,
          imu.angular_accel,
          ui.getBatteryVoltage(),
          dir == 0 ? gain * 0.1f : 0.0f,
          dir == 1 ? gain * 0.1f : 0.0f,
      });
    };
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    fan.drive(0.5);
    delay(500);
    /* start */
    if (dir == 1)
      mt.drive(-gain * 0.05f, gain * 0.05f); //< 回転
    else
      mt.drive(gain * 0.1f, gain * 0.1f); //< 並進
    for (int i = 0; i < 2000; i++) {
      sc.sampling_sync();
      printLog();
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
          bd.fbp.tra,
          bd.fbi.tra,
          bd.fbd.tra,
          sc.ref_v.rot,
          sc.est_v.rot,
          sc.ref_a.rot,
          sc.est_a.rot,
          bd.ff.rot,
          bd.fbp.rot,
          bd.fbi.rot,
          bd.fbd.rot,
      });
    };
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    fan.drive(0.2);
    delay(500);
    ctrl::AccelDesigner ad;
    if (dir == 0) {
      const float j_max = 120000;
      const float a_max = 6000;
      const float v_max = 900;
      const float dist = 90 * 4;
      ad.reset(j_max, a_max, v_max, 0, 0, dist);
    } else {
      const float j_max = 1200 * M_PI;
      const float a_max = 48 * M_PI;
      const float v_max = 6 * M_PI;
      const float dist = 4 * M_PI;
      ad.reset(j_max, a_max, v_max, 0, 0, dist);
    }
    /* start */
    sc.enable();
    for (float t = 0; t < ad.t_end() + 0.1f; t += 1e-3f) {
      if (dir == 0)
        sc.set_target(ad.v(t), 0, ad.a(t), 0);
      else
        sc.set_target(0, ad.v(t), 0, ad.a(t));
      sc.sampling_sync();
      if ((int)(t * 1000) % 2 == 0)
        printLog();
      if (mt.is_emergency())
        break;
    }
    sc.disable();
    if (mt.is_emergency())
      mt.emergency_release(), bz.play(Buzzer::EMERGENCY);
    bz.play(Buzzer::CANCEL);
  }
  static void slalom_test() {
    ctrl::TrajectoryTracker::Gain gain;
    // gain.omega_n = gain.zeta = gain.low_b = gain.low_zeta = 0;
    int mode = ui.waitForSelect(3);
    // int mode = 0;
    if (mode < 0)
      return;
    switch (mode) {
    case 0:
      break;
    case 1: {
      int value = ui.waitForSelect(16);
      if (value < 0)
        return;
      value = (value > 7) ? (value - 16) : value; //< in [-8, 7]
      gain.zeta *= std::pow(2.0f, float(value));
    } break;
    case 2: {
      int value = ui.waitForSelect(16);
      if (value < 0)
        return;
      value = (value > 7) ? (value - 16) : value; //< in [-8, 7]
      gain.omega_n *= std::pow(2.0f, float(value));
    } break;
    }
    led = 15;
    if (!ui.waitForCover())
      return;
    delay(500);
    lgr.clear();
    const auto printLog = [](const auto ref_q, const auto est_q) {
      const auto &bd = sc.fbc.getBreakdown();
      lgr.push({
#if 1
        sc.ref_v.tra, sc.est_v.tra, sc.ref_a.tra, sc.est_a.tra, bd.ff.tra,
            bd.fbp.tra, bd.fbi.tra, bd.fbd.tra, sc.ref_v.rot, sc.est_v.rot,
            sc.ref_a.rot, sc.est_a.rot, bd.ff.rot, bd.fbp.rot, bd.fbi.rot,
            bd.fbd.rot, ref_q.x, est_q.x, ref_q.y, est_q.y, ref_q.th, est_q.th,
#else
        enc.get_position(0), enc.get_position(1), imu.gyro.z, imu.accel.y,
            imu.angular_accel, ui.getBatteryVoltage(), bd.u.tra, bd.u.rot,
#endif
      });
    };
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    const auto &shape = field::shapes[field::ShapeIndex::F180];
    const float velocity = 420;
    const float Ts = 1e-3f;
    const float j_max = 120000;
    const float a_max = 6000;
    const float v_max = velocity;
    const float dist = 1 * 90;
    ctrl::TrajectoryTracker tt(gain);
    ctrl::Pose offset;
    /* start */
    sc.enable();
    tt.reset(0);
    /* accel */
    ctrl::straight::Trajectory ref;
    ref.reset(j_max, a_max, v_max, 0, velocity, dist + shape.straight_prev);
    for (float t = 0; t < ref.t_end(); t += Ts) {
      ctrl::State ref_s;
      ref.update(ref_s, t);
      const auto est_q = sc.est_p;
      const auto ref = tt.update(est_q, sc.est_v, sc.est_a, ref_s);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      sc.sampling_sync();
      printLog(ref_s.q.homogeneous(offset), est_q.homogeneous(offset));
    }
    sc.est_p.x -= ref.x_end();
    offset += ctrl::Pose(ref.x_end(), 0, 0).rotate(offset.th);
    /* slalom */
#if 1
    ctrl::slalom::Trajectory st(shape);
    st.reset(velocity);
    ctrl::State ref_s;
    for (float t = 0; t < st.getTimeCurve(); t += Ts) {
      st.update(ref_s, t, Ts);
      auto est_q = sc.est_p;
      auto ref = tt.update(est_q, sc.est_v, sc.est_a, ref_s);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      sc.sampling_sync();
      printLog(ref_s.q.homogeneous(offset), est_q.homogeneous(offset));
    }
    const auto &net = st.getShape().curve;
    sc.est_p = (sc.est_p - net).rotate(-net.th);
    offset += net.rotate(offset.th);
#endif
    /* decel */
    ref.reset(j_max, a_max, v_max, sc.ref_v.tra, 0, dist + shape.straight_post);
    for (float t = 0; t < ref.t_end(); t += Ts) {
      sc.sampling_sync();
      ctrl::State ref_s;
      ref.update(ref_s, t);
      auto est_q = sc.est_p;
      auto ref = tt.update(est_q, sc.est_v, sc.est_a, ref_s);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      printLog(ref_s.q.homogeneous(offset), est_q.homogeneous(offset));
    }
    sc.est_p.x -= ref.x_end();
    offset += ctrl::Pose(ref.x_end(), 0, 0).rotate(offset.th);
    /* ending */
    sc.disable();
    if (mt.is_emergency())
      mt.emergency_release(), bz.play(Buzzer::EMERGENCY);
    bz.play(Buzzer::CANCEL);
  }
  static void pidTuner() {
    ctrl::FeedbackController<ctrl::Polar>::Gain gain = sc.G;
    /* load */
    constexpr auto filepath = "/spiffs/machine/pid.gain";
    {
      std::ifstream f(filepath, std::ios::binary);
      if (f.fail())
        loge << "Can't open file!" << std::endl;
      else if (f.eof())
        loge << "File size is invalid!" << std::endl;
      else
        f.read((char *)(&gain), sizeof(gain));
      std::cout << filepath << std::endl;
      std::cout << "\t.Kp = ctrl::Polar(" << gain.Kp.tra << ", " << gain.Kp.rot
                << "), .Ki = ctrl::Polar(" << gain.Ki.tra << ", " << gain.Ki.rot
                << ")," << std::endl;
    }
    /* user input */
    int mode = ui.waitForSelect(5);
    int value = ui.waitForSelect(16);
    value = (value > 7) ? (value - 16) : value; //< in [-8, 7]
    if (mode < 0)
      return;
    switch (mode) {
    case 0:
      gain = sc.G;
    case 1:
      gain.Kp.tra *= std::pow(1.1f, float(value));
      break;
    case 2:
      gain.Ki.tra *= std::pow(1.1f, float(value));
      break;
    case 3:
      gain.Kp.rot *= std::pow(1.1f, float(value));
      break;
    case 4:
      gain.Ki.rot *= std::pow(1.1f, float(value));
      break;
    }
    led = 15;
    if (!ui.waitForCover())
      return;
    /* save */
    {
      std::ofstream of(filepath, std::ios::binary);
      if (of.fail())
        log_e("Can't open file!");
      else
        of.write((const char *)(&gain), sizeof(gain));
    }
    sc.G = gain;
    bz.play(Buzzer::SUCCESSFUL);
  }
  static void SearchRun_test() {
    if (!ui.waitForCover())
      return;
    delay(500);
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    ma.enqueue_action(RobotBase::START_STEP);
    ma.enqueue_action(RobotBase::TURN_R);
    ma.enqueue_action(RobotBase::ST_FULL);
    ma.enqueue_action(RobotBase::TURN_L);
    ma.enqueue_action(RobotBase::ST_HALF_STOP);
    ma.enable(MoveAction::TaskActionSearchRun);
    ma.waitForEndAction();
    ma.disable();
    mt.emergency_release();
  }
  static void position_recovery(const bool pi_enabled = false) {
    while (1) {
      led = 15;
      if (!ui.waitForCover(true))
        return;
      led = 0;
      delay(500);
      ma.enable(MoveAction::TaskActionPositionRecovery);
      ma.waitForEndAction();
      ma.disable();
      ma.emergency_release();
    }
  }
};
