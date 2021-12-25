/**
 * @file machine.h
 * @brief MircoMouse Machine
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include "app_log.h"

#include "agents/maze_robot.h"
#include "agents/move_action.h"
#include "config/io_mapping.h"
#include "config/model.h"
#include "freertospp/task.h"
#include "hardware/hardware.h"

namespace machine {

class Machine {
private:
  /* Hardware */
  hardware::Hardware *hw;
  /* Supporter */
  supporters::Supporters *sp;
  /* Agents */
  MoveAction *ma;
  MazeRobot *mr;
  /* Logger */
  Logger *lgr;

public:
  Machine() {}
  bool init() {
    /* print info */
    std::cout << std::endl;
    std::cout << "I'm KERISE v" << KERISE_SELECT << "." << std::endl;
    std::cout << "IDF Version: " << esp_get_idf_version() << std::endl;
    /* Hardware */
    hw = new hardware::Hardware();
    hw->init();
    /* Supporters */
    sp = new supporters::Supporters(hw);
    sp->init();
    /* Agents */
    ma = new MoveAction(hw, sp, model::TrajectoryTrackerGain);
    mr = new MazeRobot(hw, sp, ma);
    /* Others */
    lgr = new Logger();
    return true;
  }
  void start() {
    /* start task */
    task_drive.start(this, &Machine::task, "Drive", 4096, 1, tskNO_AFFINITY);
    task_print.start(this, &Machine::print, "Print", 4096, 1, tskNO_AFFINITY);
  }

private:
  freertospp::Task<Machine> task_drive;
  freertospp::Task<Machine> task_print;

  void task() {
    // driveAutomatically();
    while (1) {
      driveManually();
    }
  }
  void print() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(9));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(90));
      // hw->tof->print();
      // hw->ref->csv();
      // hw->enc->csv();
      // wd->print();
    }
  }
  int driveManually() {
    int mode = sp->ui->waitForSelect(16);
    switch (mode) {
    case 0: /* 迷路走行 */
      Machine::driveNormally();
      break;
    case 1: /* パラメータの相対設定 */
      Machine::selectParamManually();
      break;
    case 2: /* パラメータの絶対設定 */
      Machine::selectParamPreset();
      break;
    case 3: /* 斜め走行などの設定 */
      Machine::selectParamManual();
      break;
    case 4: /* ファンの設定 */
      Machine::selectFanGain();
      break;
    case 5: /* 迷路データの復元 */
      Machine::restore();
      break;
    case 6: /* データ消去 */
      Machine::reset();
      break;
    case 7: /* 宴会芸 */
      Machine::partyStunt();
      break;
    case 8: /* 壁センサキャリブレーション */
      Machine::wallCalibration();
      break;
    case 9: /* プチコン */
      Machine::petitcon();
      break;
    case 10: /* 迷路の表示 */
      mr->print();
      break;
    case 11: /* ゴール区画の設定 */
      Machine::setGoalPositions();
      break;
    case 12:
      Machine::position_recovery();
      // Machine::sysid();
      break;
    case 13:
      // Machine::pidTuner();
      // Machine::encoder_test();
      // Machine::accel_test();
      Machine::wall_attach_test();
      break;
    case 14: /* テスト */
      Machine::slalom_test();
      break;
    case 15: /* ログの表示 */
      lgr->print();
      break;
    }
    return 0;
  }
  void driveAutomatically() {
    restore();
    if (sp->ui->waitForPickup())
      return;
    mr->autoRun();
  }
  void driveNormally() {
    /* 探索状態のお知らせ */
    if (mr->getMaze().getWallRecords().empty()) //< 完全に未探索状態
      hw->bz->play(hardware::Buzzer::CONFIRM);
    else if (mr->isComplete()) //< 完全に探索終了
      hw->bz->play(hardware::Buzzer::SUCCESSFUL);
    else if (mr->calcShortestDirections(true)) //< 探索中だが経路はある
      hw->bz->play(hardware::Buzzer::MAZE_RESTORE);
    else //< 行きの探索中
      hw->bz->play(hardware::Buzzer::MAZE_BACKUP);
    /* 異常検出 */
    if (!mr->isSolvable()) {
      hw->bz->play(hardware::Buzzer::ERROR);
      mr->resetLastWalls(6);
      return;
    }
    /* 走行オプション */
    int mode = sp->ui->waitForSelect(2);
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
    if (!sp->ui->waitForCover())
      return;
    hw->led->set(9);
    // vTaskDelay(pdMS_TO_TICKS(5000)); //< 動画用 delay
    mr->autoRun(forceSearch);
  }
  void restore() {
    if (!mr->restore())
      hw->bz->play(hardware::Buzzer::ERROR);
    else
      hw->bz->play(hardware::Buzzer::MAZE_RESTORE);
    /* ゴールが封印されていないか一応確認 */
    if (!mr->isSolvable())
      hw->bz->play(hardware::Buzzer::ERROR);
  }
  void reset() {
    if (!sp->ui->waitForCover())
      return;
    hw->bz->play(hardware::Buzzer::MAZE_BACKUP);
    mr->reset();
  }
  void selectParamPreset() {
    for (int i = 0; i < 1; i++)
      hw->bz->play(hardware::Buzzer::SHORT7);
    int value = sp->ui->waitForSelect(16);
    if (value < 0)
      return;
    ma->rp_fast = MoveAction::RunParameter();
    if (value <= 7)
      ma->rp_fast.up(value);
    else
      ma->rp_fast.down(16 - value);
    hw->bz->play(hardware::Buzzer::SUCCESSFUL);
  }
  void selectParamManual() {
    int mode = sp->ui->waitForSelect(16);
    if (mode < 0)
      return;
    int value = sp->ui->waitForSelect(4);
    if (value < 0)
      return;
    switch (mode) {
    case 0: /* 斜め走行 */
      ma->rp_search.diag_enabled = value & 0x01;
      ma->rp_fast.diag_enabled = value & 0x02;
      break;
    case 1: /* 未知区間加速 */
      ma->rp_search.unknown_accel_enabled = value & 0x01;
      ma->rp_fast.unknown_accel_enabled = value & 0x02;
      break;
    case 2: /* 前壁修正 */
      ma->rp_search.front_wall_fix_enabled = value & 1;
      ma->rp_fast.front_wall_fix_enabled = value & 2;
      break;
    case 3: /* 横壁補正 */
      ma->rp_search.wall_avoid_enabled = value & 1;
      ma->rp_fast.wall_avoid_enabled = value & 2;
      break;
    case 4: /* 横壁姿勢補正 */
      ma->rp_search.wall_theta_fix_enabled = value & 1;
      ma->rp_fast.wall_theta_fix_enabled = value & 2;
      break;
    case 5: /* 壁切れ */
      ma->rp_search.wall_cut_enabled = value & 1;
      ma->rp_fast.wall_cut_enabled = value & 2;
      break;
    }
    hw->bz->play(hardware::Buzzer::SUCCESSFUL);
  }
  void selectParamManually() {
    int value;
    /* ターン速度 */
    for (int i = 0; i < 1; i++)
      hw->bz->play(hardware::Buzzer::SHORT7);
    value = sp->ui->waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    for (auto &vs : ma->rp_fast.v_slalom)
      vs *= std::pow(ma->rp_fast.vs_factor, float(value));
    /* 最大速度 */
    for (int i = 0; i < 2; i++)
      hw->bz->play(hardware::Buzzer::SHORT7);
    value = sp->ui->waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    ma->rp_fast.v_max *= std::pow(ma->rp_fast.vm_factor, float(value));
    /* 加速度 */
    for (int i = 0; i < 3; i++)
      hw->bz->play(hardware::Buzzer::SHORT7);
    value = sp->ui->waitForSelect(16);
    if (value < 0)
      return;
    if (value > 7)
      value -= 16;
    ma->rp_fast.a_max *= std::pow(ma->rp_fast.vm_factor, float(value));
    /* 成功 */
    hw->bz->play(hardware::Buzzer::SUCCESSFUL);
  }
  void selectFanGain() {
    hw->fan->drive(0.5f);
    vTaskDelay(pdMS_TO_TICKS(100));
    hw->fan->drive(0);
    int value = sp->ui->waitForSelect(11);
    if (value < 0)
      return;
    ma->rp_fast.fan_duty = 0.1f * value;
    hw->fan->drive(ma->rp_fast.fan_duty);
    // hw->mt->drive(ma->fan_duty, ma->fan_duty);
    sp->ui->waitForSelect(1);
    hw->fan->drive(0);
    hw->mt->drive(0, 0);
    hw->bz->play(hardware::Buzzer::SUCCESSFUL);
  }
  void wallCalibration() {
    int mode = sp->ui->waitForSelect(3);
    switch (mode) {
    /* 前壁補正データの保存 */
    case 0:
      hw->led->set(15);
      if (!sp->ui->waitForCover())
        return;
      if (sp->wd->backup()) {
        hw->bz->play(hardware::Buzzer::SUCCESSFUL);
      } else {
        hw->bz->play(hardware::Buzzer::ERROR);
      }
      break;
    /* 横壁キャリブレーション */
    case 1:
      hw->led->set(9);
      if (!sp->ui->waitForCover())
        return;
      vTaskDelay(pdMS_TO_TICKS(1000));
      hw->bz->play(hardware::Buzzer::CONFIRM);
      sp->wd->calibration_side();
      hw->bz->play(hardware::Buzzer::CANCEL);
      break;
    /* 前壁キャリブレーション */
    case 2:
      hw->led->set(6);
      if (!sp->ui->waitForCover(true))
        return;
      vTaskDelay(pdMS_TO_TICKS(1000));
      hw->bz->play(hardware::Buzzer::CONFIRM);
      sp->wd->calibration_front();
      hw->bz->play(hardware::Buzzer::CANCEL);
      break;
    }
  }
  void setGoalPositions() {
    for (int i = 0; i < 2; i++)
      hw->bz->play(hardware::Buzzer::SHORT7);
    int value = sp->ui->waitForSelect(6);
    if (value < 0)
      return;
    switch (value) {
    case 0: {
      int x = sp->ui->waitForSelect(16);
      int y = sp->ui->waitForSelect(16);
      mr->setGoals({MazeLib::Position(x, y)});
      break;
    }
    case 1:
      mr->setGoals({MazeLib::Position(1, 0)});
      break;
    case 2:
      mr->setGoals({
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
      mr->setGoals({MazeLib::Position(3, 3), MazeLib::Position(3, 4),
                    MazeLib::Position(4, 3), MazeLib::Position(4, 4)});
      break;
    case 4:
      mr->setGoals({MazeLib::Position(8, 8)});
      break;
    case 5:
      mr->setGoals({MazeLib::Position(15, 15)});
      break;
    }
    hw->bz->play(hardware::Buzzer::SUCCESSFUL);
  }
  void partyStunt() {
    if (!sp->ui->waitForCover())
      return;
    hw->led->set(6);
    hw->bz->play(hardware::Buzzer::CALIBRATION);
    hw->imu->calibration();
    hw->led->set(9);
    sp->sc->enable();
    sp->sc->set_target(0, 0);
    while (!hw->mt->is_emergency())
      vTaskDelay(pdMS_TO_TICKS(1));
    hw->mt->drive(0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    sp->sc->disable();
    hw->mt->emergency_release();
    hw->bz->play(hardware::Buzzer::CANCEL);
  }
  void petitcon() {
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(500));
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
    ma->set_fast_path(path);
    ma->enable(MoveAction::TaskActionFastRun);
    ma->waitForEndAction();
    ma->disable();
    ma->emergency_release();
  }
  void sysid() {
    int dir = sp->ui->waitForSelect(2);
    int gain = sp->ui->waitForSelect();
    if (gain < 0)
      return;
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(1000));
    lgr->clear();
    const auto printLog = [&]() {
      lgr->push({
          hw->enc->get_position(0),
          hw->enc->get_position(1),
          hw->imu->gyro.z,
          hw->imu->accel.y,
          hw->imu->angular_accel,
          sp->ui->getBatteryVoltage(),
          dir == 0 ? gain * 0.1f : 0.0f,
          dir == 1 ? gain * 0.1f : 0.0f,
      });
    };
    hw->bz->play(hardware::Buzzer::CALIBRATION);
    hw->imu->calibration();
    hw->fan->drive(0.5);
    vTaskDelay(pdMS_TO_TICKS(500));
    /* start */
    if (dir == 1)
      hw->mt->drive(-gain * 0.05f, gain * 0.05f); //< 回転
    else
      hw->mt->drive(gain * 0.1f, gain * 0.1f); //< 並進
    for (int i = 0; i < 2000; i++) {
      sp->sc->sampling_sync();
      printLog();
    }
    hw->fan->drive(0);
    hw->mt->drive(0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    hw->mt->free();
  }
  void accel_test() {
    int dir = sp->ui->waitForSelect(2);
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(500));
    lgr->clear();
    const auto printLog = [this]() {
      const auto &bd = sp->sc->fbc.getBreakdown();
      lgr->push({
        sp->sc->ref_v.tra, sp->sc->est_v.tra, sp->sc->ref_a.tra,
            sp->sc->est_a.tra, bd.ff.tra, bd.fbp.tra, bd.fbi.tra, bd.fbd.tra,
            sp->sc->ref_v.rot, sp->sc->est_v.rot, sp->sc->ref_a.rot,
            sp->sc->est_a.rot, bd.ff.rot, bd.fbp.rot, bd.fbi.rot, bd.fbd.rot,
#if 0
            (float)hw->enc->get_raw(0), (float)hw->enc->get_raw(1),
#endif
      });
    };
    hw->bz->play(hardware::Buzzer::CALIBRATION);
    hw->imu->calibration();
    hw->fan->drive(0.2);
    vTaskDelay(pdMS_TO_TICKS(500));
    ctrl::AccelDesigner ad;
    if (dir == 0) {
      const float j_max = 240'000;
      const float a_max = 9000;
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
    sp->sc->enable();
    for (float t = 0; t < ad.t_end() + 0.1f; t += 1e-3f) {
      if (dir == 0)
        sp->sc->set_target(ad.v(t), 0, ad.a(t), 0);
      else
        sp->sc->set_target(0, ad.v(t), 0, ad.a(t));
      sp->sc->sampling_sync();
      if ((int)(t * 1000) % 2 == 0)
        printLog();
      if (hw->mt->is_emergency())
        break;
    }
    sp->sc->disable();
    if (hw->mt->is_emergency())
      hw->mt->emergency_release(), hw->bz->play(hardware::Buzzer::EMERGENCY);
    hw->bz->play(hardware::Buzzer::CANCEL);
  }
  void slalom_test() {
    ctrl::TrajectoryTracker::Gain gain = model::TrajectoryTrackerGain;
    // gain.omega_n = gain.zeta = gain.low_b = gain.low_zeta = 0;
    int mode = sp->ui->waitForSelect(3);
    // int mode = 0;
    if (mode < 0)
      return;
    // switch (mode) {
    // case 0:
    //   break;
    // case 1: {
    //   int value = sp->ui->waitForSelect(16);
    //   if (value < 0)
    //     return;
    //   value = (value > 7) ? (value - 16) : value; //< in [-8, 7]
    //   gain.zeta *= std::pow(2.0f, float(value));
    // } break;
    // case 2: {
    //   int value = sp->ui->waitForSelect(16);
    //   if (value < 0)
    //     return;
    //   value = (value > 7) ? (value - 16) : value; //< in [-8, 7]
    //   gain.omega_n *= std::pow(2.0f, float(value));
    // } break;
    // }
    hw->led->set(15);
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(500));
    lgr->clear();
    const auto printLog = [this](const auto ref_q, const auto est_q) {
      const auto &bd = sp->sc->fbc.getBreakdown();
      lgr->push({
#if 1
        sp->sc->ref_v.tra, sp->sc->est_v.tra, sp->sc->ref_a.tra,
            sp->sc->est_a.tra, bd.ff.tra, bd.fbp.tra, bd.fbi.tra, bd.fbd.tra,
            sp->sc->ref_v.rot, sp->sc->est_v.rot, sp->sc->ref_a.rot,
            sp->sc->est_a.rot, bd.ff.rot, bd.fbp.rot, bd.fbi.rot, bd.fbd.rot,
            ref_q.x, est_q.x, ref_q.y, est_q.y, ref_q.th, est_q.th,
#else
        hw->enc->get_position(0), hw->enc->get_position(1), hw->imu->gyro.z,
            hw->imu->accel.y, hw->imu->angular_accel, bd.u.tra, bd.u.rot,
#endif
      });
    };
    hw->bz->play(hardware::Buzzer::CALIBRATION);
    hw->imu->calibration();
    const auto &shape = field::shapes[field::ShapeIndex::F180];
    const bool mirror = mode;
    const float velocity = 450;
    const float Ts = 1e-3f;
    const float j_max = 120'000;
    const float a_max = 6000;
    const float v_max = velocity;
    const float d_1 = 1 * 45;
    const float d_2 = 1 * 45;
    ctrl::TrajectoryTracker tt(gain);
    ctrl::Pose offset;
    /* start */
    sp->sc->enable();
    tt.reset(0);
    /* accel */
    ctrl::straight::Trajectory ref;
    ref.reset(j_max, a_max, v_max, 0, velocity, d_1 + shape.straight_prev);
    for (float t = 0; t < ref.t_end(); t += Ts) {
      ctrl::State ref_s;
      ref.update(ref_s, t);
      const auto est_q = sp->sc->est_p;
      const auto ref = tt.update(est_q, sp->sc->est_v, sp->sc->est_a, ref_s);
      sp->sc->set_target(ref.v, ref.w, ref.dv, ref.dw);
      sp->sc->sampling_sync();
      printLog(ref_s.q.homogeneous(offset), est_q.homogeneous(offset));
    }
    sp->sc->est_p.x -= ref.x_end();
    offset += ctrl::Pose(ref.x_end(), 0, 0).rotate(offset.th);
    /* slalom */
    ctrl::slalom::Trajectory st(shape, mirror);
    st.reset(velocity);
    ctrl::State ref_s;
    for (float t = 0; t < st.getTimeCurve(); t += Ts) {
      st.update(ref_s, t, Ts);
      auto est_q = sp->sc->est_p;
      auto ref = tt.update(est_q, sp->sc->est_v, sp->sc->est_a, ref_s);
      sp->sc->set_target(ref.v, ref.w, ref.dv, ref.dw);
      sp->sc->sampling_sync();
      printLog(ref_s.q.homogeneous(offset), est_q.homogeneous(offset));
    }
    const auto &net = st.getShape().curve;
    sp->sc->est_p = (sp->sc->est_p - net).rotate(-net.th);
    offset += net.rotate(offset.th);
    /* decel */
    ref.reset(j_max, a_max, v_max, sp->sc->ref_v.tra, 0,
              d_2 + shape.straight_post);
    for (float t = 0; t < ref.t_end(); t += Ts) {
      ctrl::State ref_s;
      ref.update(ref_s, t);
      auto est_q = sp->sc->est_p;
      auto ref = tt.update(est_q, sp->sc->est_v, sp->sc->est_a, ref_s);
      sp->sc->set_target(ref.v, ref.w, ref.dv, ref.dw);
      sp->sc->sampling_sync();
      printLog(ref_s.q.homogeneous(offset), est_q.homogeneous(offset));
    }
    sp->sc->est_p.x -= ref.x_end();
    offset += ctrl::Pose(ref.x_end(), 0, 0).rotate(offset.th);
    /* ending */
    sp->sc->disable();
    if (hw->mt->is_emergency())
      hw->mt->emergency_release(), hw->bz->play(hardware::Buzzer::EMERGENCY);
    hw->bz->play(hardware::Buzzer::CANCEL);
  }
  void pidTuner() {
    ctrl::FeedbackController<ctrl::Polar>::Gain gain = sp->sc->fbc.getGain();
    /* load */
    constexpr auto filepath = "/spiffs/machine/pid.gain";
    {
      std::ifstream f(filepath, std::ios::binary);
      if (f.fail())
        app_loge << "Can't open file!" << std::endl;
      else if (f.eof())
        app_loge << "File size is invalid!" << std::endl;
      else
        f.read((char *)(&gain), sizeof(gain));
      std::cout << filepath << std::endl;
      std::cout << "\t.Kp = ctrl::Polar(" << gain.Kp.tra << ", " << gain.Kp.rot
                << "), .Ki = ctrl::Polar(" << gain.Ki.tra << ", " << gain.Ki.rot
                << ")," << std::endl;
    }
    /* user input */
    int mode = sp->ui->waitForSelect(5);
    int value = sp->ui->waitForSelect(16);
    value = (value > 7) ? (value - 16) : value; //< in [-8, 7]
    if (mode < 0)
      return;
    switch (mode) {
    case 0:
      gain = sp->sc->fbc.getGain();
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
    hw->led->set(15);
    if (!sp->ui->waitForCover())
      return;
    /* save */
    std::ofstream of(filepath, std::ios::binary);
    if (of.fail()) {
      app_loge << "Can't open file. " << filepath << std::endl;
      hw->bz->play(hardware::Buzzer::ERROR);
    } else {
      of.write((const char *)(&gain), sizeof(gain));
    }
    sp->sc->fbc.setGain(gain);
    hw->bz->play(hardware::Buzzer::SUCCESSFUL);
  }
  void SearchRun_test() {
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(500));
    hw->bz->play(hardware::Buzzer::CALIBRATION);
    hw->imu->calibration();
    ma->enqueue_action(RobotBase::START_STEP);
    ma->enqueue_action(RobotBase::TURN_R);
    ma->enqueue_action(RobotBase::ST_FULL);
    ma->enqueue_action(RobotBase::TURN_L);
    ma->enqueue_action(RobotBase::ST_HALF_STOP);
    ma->enable(MoveAction::TaskActionSearchRun);
    ma->waitForEndAction();
    ma->disable();
    hw->mt->emergency_release();
  }
  void position_recovery() {
    while (1) {
      hw->led->set(15);
      if (!sp->ui->waitForCover(true))
        return;
      hw->led->set(0);
      vTaskDelay(pdMS_TO_TICKS(500));
      ma->enable(MoveAction::TaskActionPositionRecovery);
      ma->waitForEndAction();
      ma->disable();
      ma->emergency_release();
    }
  }
  void encoder_test() {
    int value = sp->ui->waitForSelect(16);
    hw->led->set(15);
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(500));
    float pwm = 0.05 * value;
    hw->mt->drive(pwm, pwm);
    sp->ui->waitForCover(true);
    hw->mt->free();
  }
  void wall_attach_test() {
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(500));
    sp->sc->enable();
    //
    hw->led->set(6);
    hw->tof->disable();
    vTaskDelay(pdMS_TO_TICKS(20)); /*< ノイズ防止のためToFを無効化 */
    while (1) {
      if (hw->mt->is_emergency()) {
        hw->bz->play(hardware::Buzzer::EMERGENCY);
        break;
      }
      if (hw->ref->side(0) > 1.2 * UserInterface::thr_ref_side &&
          hw->ref->side(1) > 1.2 * UserInterface::thr_ref_side) {
        hw->bz->play(hardware::Buzzer::CANCEL);
        break;
      }
      sp->sc->sampling_sync();
      WheelParameter wp;
      for (int j = 0; j < 2; ++j)
        wp.wheel[j] = -sp->wd->distance.front[j] * model::wall_attach_gain_Kp;
      const float end = model::wall_attach_end;
      if (math_utils::sum_of_square(wp.wheel[0], wp.wheel[1]) < end)
        hw->led->set(0);
      else
        hw->led->set(6);
      wp.wheel2pole();
      const float sat_tra = 180.0f;   //< [mm/s]
      const float sat_rot = 1 * M_PI; //< [rad/s]
      sp->sc->set_target(math_utils::saturate(wp.tra, sat_tra),
                         math_utils::saturate(wp.rot, sat_rot));
    }
    sp->sc->set_target(0, 0);
    hw->tof->enable();
    hw->led->set(0);
    //
    sp->sc->disable();
    hw->mt->emergency_release();
  }
};

} // namespace machine
