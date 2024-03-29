/**
 * @file machine.h
 * @brief MircoMouse Machine
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include "agents/maze_robot.h"
#include "freertospp/task.h"
#include "peripheral/esp.h"
#include "peripheral/spiffs.h"

namespace machine {

class Machine {
 private:
  void driveManually() {
    int mode = sp->ui->waitForSelect(16);
    switch (mode) {
      case 0: /* 探索・最短を含む通常の自律走行 */
        Machine::driveNormally();
        break;
      case 1: /* パラメータの相対設定 */
        Machine::selectParamManually();
        break;
      case 2: /* パラメータの絶対設定 */
        Machine::selectParamPreset();
        break;
      case 3: /* 走行設定 */
        Machine::selectRunConfig();
        break;
      case 4: /* ファン設定 */
        Machine::selectFanGain();
        break;
      case 5: /* 迷路データ・探索状態の復元・削除 */
        Machine::selectBackupData();
        break;
      case 6: /* ゴール区画の設定 */
        Machine::setGoalPositions();
        break;
      case 7: /* 宴会芸 */
        Machine::partyStunt();
        break;
      case 8: /* 壁センサキャリブレーション */
        Machine::wallCalibration();
        break;
      case 9: /* 壁センサのリアルタイム表示 */
        Machine::print_wall_detector();
        break;
      case 10: /* 迷路情報の表示 */
        mr->print();
        break;
      case 11: /* プチコン */
        Machine::petitcon();
        break;
      case 12: /* タイヤ径の測定 */
        Machine::wheel_diameter_measurement();
        break;
      case 13:
        // Machine::encoder_test();
        // Machine::accel_test();
        Machine::front_wall_attach_test();
        // Machine::position_recovery();
        // Machine::sysid();
        // Machine::wall_test();
        break;
      case 14: /* テスト */
        Machine::slalom_test();
        break;
      case 15: /* ログの表示 */
        lgr->print();
        break;
    }
  }
  void driveAutomatically() {
    /* 回収待ち */
    if (sp->ui->waitForPickup())
      return;
    /* 状態復元 */
    Machine::restore();
    /* 自己位置復帰 */
    mr->autoRun(true, true);
  }
  void driveNormally() {
    /* 探索状態のお知らせ */
    if (mr->getMaze().getWallRecords().empty())  //< 完全に未探索状態
      hw->bz->play(hardware::Buzzer::CONFIRM);
    else if (mr->isComplete())  //< 完全に探索終了
      hw->bz->play(hardware::Buzzer::SUCCESSFUL);
    else if (mr->calcShortestDirections(true))  //< 探索中だが経路はある
      hw->bz->play(hardware::Buzzer::MAZE_RESTORE);
    else  //< 行きの探索中
      hw->bz->play(hardware::Buzzer::MAZE_BACKUP);
    /* 異常検出 */
    if (!mr->isSolvable()) {
      hw->bz->play(hardware::Buzzer::ERROR);
      mr->resetLastWalls(3);
      return;
    }
    /* 走行オプション */
    int mode = sp->ui->waitForSelect(2);
    if (mode < 0)
      return;
    bool isAutoParamSelect = true;
    switch (mode) {
      case 0: /*< デフォルト */
        break;
      case 1: /*< パラメータ固定 */
        isAutoParamSelect = false;
        break;
    }
    if (!sp->ui->waitForCover())
      return;
    hw->led->set(9);
    // vTaskDelay(pdMS_TO_TICKS(5000)); //< 動画用 delay
    mr->autoRun(isAutoParamSelect);
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
    for (auto& vs : ma->rp_fast.v_slalom)
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
  void selectParamPreset() {
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
  void selectRunConfig() {
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
      case 2: /* 前壁補正 */
        ma->rp_search.front_wall_fix_enabled = value & 1;
        ma->rp_fast.front_wall_fix_enabled = value & 2;
        break;
      case 3: /* 横壁補正 */
        ma->rp_search.side_wall_avoid_enabled = value & 1;
        ma->rp_fast.side_wall_avoid_enabled = value & 2;
        break;
      case 4: /* 横壁姿勢補正 */
        ma->rp_search.side_wall_fix_theta_enabled = value & 1;
        ma->rp_fast.side_wall_fix_theta_enabled = value & 2;
        break;
      case 5: /* V90の横壁補正 */
        ma->rp_search.side_wall_fix_v90_enabled = value & 1;
        ma->rp_fast.side_wall_fix_v90_enabled = value & 2;
        break;
      case 6: /* 壁切れ */
        ma->rp_search.side_wall_cut_enabled = value & 1;
        ma->rp_fast.side_wall_cut_enabled = value & 2;
        break;
      case 7: /* 探索速度 */
        value = (value < 3) ? 12 : value;
        ma->rp_search.v_search = ma->rp_fast.v_search = 30 * value;
        break;
    }
    hw->bz->play(hardware::Buzzer::SUCCESSFUL);
  }
  void selectFanGain() {
    /* 吸引ファンの設定であることをお知らせ */
    hw->fan->drive(0.5f);
    vTaskDelay(pdMS_TO_TICKS(100));
    hw->fan->drive(0);
    /* 設定値の取得 */
    int value = sp->ui->waitForSelect(11);
    if (value < 0)
      return;
    /* 吸引ファンの駆動テスト */
    const float fan_duty = 0.1f * value;
    hw->fan->drive(fan_duty);
    bool res = sp->ui->waitForCover();
    hw->fan->drive(0);
    if (!res)
      return;
    /* 設定の反映 */
    ma->rp_fast.fan_duty = fan_duty;
    hw->bz->play(hardware::Buzzer::SUCCESSFUL);
  }
  void selectBackupData() {
    int mode = sp->ui->waitForSelect(2);
    if (mode < 0)
      return;
    /* wait for confirm */
    if (!sp->ui->waitForCover())
      return;
    /* apply */
    switch (mode) {
      case 0: /* restore */
        restore();
        break;
      case 1: /* reset */
        reset();
        break;
    }
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
    hw->bz->play(hardware::Buzzer::BOOT);
    mr->reset();
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
    int value = sp->ui->waitForSelect(16);
    if (value < 0)
      return;
    switch (value) {
      case 0: {  //< マニュアル選択
        int x = sp->ui->waitForSelect(16);
        int y = sp->ui->waitForSelect(16);
        mr->setGoals({MazeLib::Position(x, y)});
        break;
      }
      case 1:  //< 調整迷路
        mr->setGoals({MazeLib::Position(1, 0)});
        mr->setTimeout(9);
        break;
      case 2:  //< 調整迷路
        mr->setGoals({MazeLib::Position(8, 8)});
        mr->setTimeout(16);
        break;
      case 15:  //< 本番迷路
        mr->setGoals({
            MazeLib::Position(16, 16),
            MazeLib::Position(16, 17),
            MazeLib::Position(16, 18),
            MazeLib::Position(17, 16),
            MazeLib::Position(17, 17),
            MazeLib::Position(17, 18),
            MazeLib::Position(18, 16),
            MazeLib::Position(18, 17),
            MazeLib::Position(18, 18),
        });
        mr->setTimeout(32);
        break;
      default:
        mr->setGoals({MazeLib::Position(value, value)});
        break;
    }
    hw->bz->play(hardware::Buzzer::SUCCESSFUL);
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

 private:
  enum LOG_SELECT {
    LOG_PID,
    LOG_SYSID,
    LOG_WALL,
  };
  void log_init(enum LOG_SELECT log_select) {
    switch (log_select) {
      case LOG_PID:
        return lgr->init({
            "ref_v.tra", "est_v.tra", "ref_a.tra", "est_a.tra", "ff.tra",
            "fbp.tra",   "fbi.tra",   "fbd.tra",   "ref_v.rot", "est_v.rot",
            "ref_a.rot", "est_a.rot", "ff.rot",    "fbp.rot",   "fbi.rot",
            "fbd.rot",   "ref_q.x",   "est_q.x",   "ref_q.y",   "est_q.y",
            "ref_q.th",  "est_q.th",
        });
      case LOG_SYSID:
        return lgr->init({
            "enc[0]",
            "enc[1]",
            "gyro.z",
            "accel.y",
            "angular_accel",
            "u.tra",
            "u.rot",
        });
      case LOG_WALL:
        return lgr->init({
            "ref_v.tra", "est_v.tra", "ref_a.tra", "est_a.tra", "ref_q.x",
            "est_q.x",   "ref_q.y",   "est_q.y",   "ref_q.th",  "est_q.th",
            "ref_0",     "ref_1",     "ref_2",     "ref_3",     "wd_0",
            "wd_1",      "wd_2",      "wd_3",      "tof",
        });
    }
  }
  void log_push(enum LOG_SELECT log_select,
                const ctrl::Pose& ref_q = ctrl::Pose(),
                const ctrl::Pose& est_q = ctrl::Pose()) {
    const auto& bd = sp->sc->fbc.getBreakdown();
    switch (log_select) {
      case LOG_PID:
        return lgr->push({
            sp->sc->ref_v.tra, sp->sc->est_v.tra, sp->sc->ref_a.tra,
            sp->sc->est_a.tra, bd.ff.tra,         bd.fbp.tra,
            bd.fbi.tra,        bd.fbd.tra,        sp->sc->ref_v.rot,
            sp->sc->est_v.rot, sp->sc->ref_a.rot, sp->sc->est_a.rot,
            bd.ff.rot,         bd.fbp.rot,        bd.fbi.rot,
            bd.fbd.rot,        ref_q.x,           est_q.x,
            ref_q.y,           est_q.y,           ref_q.th,
            est_q.th,
        });
      case LOG_SYSID:
        return lgr->push({
            hw->enc->get_position(0),
            hw->enc->get_position(1),
            hw->imu->get_gyro(),
            hw->imu->get_accel(),
            hw->imu->get_angular_accel(),
            bd.u.tra,
            bd.u.rot,
        });
      case LOG_WALL:
        return lgr->push({
            sp->sc->ref_v.tra,
            sp->sc->est_v.tra,
            sp->sc->ref_v.rot,
            sp->sc->est_v.rot,
            ref_q.x,
            est_q.x,
            ref_q.y,
            est_q.y,
            ref_q.th,
            est_q.th,
            (float)hw->rfl->side(0),
            (float)hw->rfl->front(0),
            (float)hw->rfl->front(1),
            (float)hw->rfl->side(1),
            (float)sp->wd->distance.side[0],
            (float)sp->wd->distance.front[0],
            (float)sp->wd->distance.front[1],
            (float)sp->wd->distance.side[1],
            (float)hw->tof->getDistance(),
        });
    }
  }
  void sysid() {
    int dir = sp->ui->waitForSelect(2);
    int gain = sp->ui->waitForSelect();
    if (gain < 0)
      return;
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(1000));
    lgr->init({
        "enc[0]",
        "enc[1]",
        "gyro.z",
        "accel.y",
        "angular_accel",
        "u.tra",
        "u.rot",
        "battery_voltage",
    });
    const auto push_log = [&]() {
      const auto& bd = sp->sc->fbc.getBreakdown();
      lgr->push({
          hw->enc->get_position(0), hw->enc->get_position(1),
          hw->imu->get_gyro(), hw->imu->get_accel(),
          hw->imu->get_angular_accel(), bd.u.tra, bd.u.rot,
          // sp->ui->getBatteryVoltage(),
      });
    };
    hw->bz->play(hardware::Buzzer::CALIBRATION);
    hw->imu->calibration();
    hw->fan->drive(0.5);
    vTaskDelay(pdMS_TO_TICKS(500));
    /* start */
    if (dir == 1)
      hw->mt->drive(-gain * 0.05f, gain * 0.05f);  //< 回転
    else
      hw->mt->drive(gain * 0.1f, gain * 0.1f);  //< 並進
    for (int i = 0; i < 2000; i++) {
      sp->sc->sampling_sync();
      push_log();
    }
    hw->fan->drive(0);
    hw->mt->drive(0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    hw->mt->free();
  }
  void wheel_diameter_measurement() {
    int cells = sp->ui->waitForSelect(16);
    if (cells < 0)
      return;
    cells = (cells == 0) ? 8 : cells;  //< default is 8 cells straight
    while (1) {
      /* wait for start */
      if (!sp->ui->waitForCover())
        return;
      vTaskDelay(pdMS_TO_TICKS(500));
      /* calibration */
      hw->bz->play(hardware::Buzzer::CALIBRATION);
      hw->imu->calibration();
      /* put back */
      hw->mt->drive(-0.1f, -0.1f);
      vTaskDelay(pdMS_TO_TICKS(200));
      /* config */
      float dist = 90 * cells - (model::TailLength + field::WallThickness / 2);
      ctrl::AccelDesigner ad;
      ad.reset(120'000, 3'000, 720, 0, 30, dist);
      /* start */
      sp->sc->enable();  //< includes position reset
      for (float t = 0; !hw->mt->is_emergency(); t += 1e-3f) {
        sp->sc->set_target(ad.v(t), 0, ad.a(t), 0);
        sp->sc->sampling_sync();
        if (sp->sc->est_p.x > dist)
          break;
      }
      /* stop statically */
      sp->sc->set_target(0, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
      sp->sc->disable();
      if (hw->mt->is_emergency())
        hw->mt->emergency_release(), hw->bz->play(hardware::Buzzer::EMERGENCY);
      hw->bz->play(hardware::Buzzer::CANCEL);
      /* wait for next */
      if (sp->ui->waitForSelect(1) < 0)
        return;
    }
  }
  void wall_test() {
    int cells = sp->ui->waitForSelect(16);
    if (cells < 0)
      return;
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(500));
    /* config */
    const float j_max = 240'000;
    const float a_max = 9000;
    const float v_max = 600;
    const float dist = field::SegWidthFull * cells;
    enum LOG_SELECT log_select = LOG_WALL;
    /* prepare */
    log_init(log_select);
    hw->bz->play(hardware::Buzzer::CALIBRATION);
    hw->imu->calibration();
    ctrl::AccelDesigner ad;
    ad.reset(j_max, a_max, v_max, 0, 0, dist);
    /* start */
    sp->sc->enable();
    for (float t = 0; t < ad.t_end() + 0.1f; t += 1e-3f) {
      sp->sc->set_target(ad.v(t), 0, ad.a(t), 0);
      sp->sc->sampling_sync();
      // if ((int)(t * 1000) % 2 == 0)
      log_push(log_select, ctrl::Pose(ad.x(t)), sp->sc->est_p);
      if (hw->mt->is_emergency())
        break;
    }
    /* end */
    sp->sc->disable();
    if (hw->mt->is_emergency())
      hw->mt->emergency_release(), hw->bz->play(hardware::Buzzer::EMERGENCY);
    hw->bz->play(hardware::Buzzer::CANCEL);
  }
  void accel_test() {
    int cells = sp->ui->waitForSelect(2);
    if (cells < 0)
      return;
    int dir = sp->ui->waitForSelect(2);
    if (dir < 0)
      return;
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(500));
    enum LOG_SELECT log_select = LOG_PID;
    log_init(log_select);
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
      const float j_max = 1200 * PI;
      const float a_max = 48 * PI;
      const float v_max = 6 * PI;
      const float dist = 4 * PI;
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
        log_push(log_select,
                 dir == 0 ? ctrl::Pose(ad.x(t)) : ctrl::Pose(0, 0, ad.x(t)),
                 sp->sc->est_p);
      if (hw->mt->is_emergency())
        break;
    }
    sp->sc->disable();
    if (hw->mt->is_emergency())
      hw->mt->emergency_release(), hw->bz->play(hardware::Buzzer::EMERGENCY);
    hw->bz->play(hardware::Buzzer::CANCEL);
  }
  void slalom_test() {
    int mode = sp->ui->waitForSelect(2);
    if (mode < 0)
      return;
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(500));
    enum LOG_SELECT log_select = LOG_PID;
    log_init(log_select);
    hw->bz->play(hardware::Buzzer::CALIBRATION);
    hw->imu->calibration();
    hw->enc->clear_offset();
    /* parameter */
    const auto& shape = field::shapes[field::ShapeIndex::F180];
    const bool mirror = mode;
    const float velocity = 600;
    const float Ts = 1e-3f;
    const float j_max = 240'000;
    const float a_max = 6000;
    const float v_max = velocity;
    const float d_1 = 2 * 45;
    const float d_2 = 2 * 45;
    const float fan_duty = 0.0;
    ctrl::TrajectoryTracker tt(model::TrajectoryTrackerGain);
    ctrl::Pose offset;
    /* fan */
    if (fan_duty > 0) {
      hw->fan->drive(fan_duty);
      vTaskDelay(pdMS_TO_TICKS(400));
    }
    /* start */
    sp->sc->enable();
    tt.reset(/* v_start = */ 0);
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
      log_push(log_select, ref_s.q.homogeneous(offset),
               est_q.homogeneous(offset));
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
      log_push(log_select, ref_s.q.homogeneous(offset),
               est_q.homogeneous(offset));
    }
    const auto& net = st.getShape().curve;
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
      log_push(log_select, ref_s.q.homogeneous(offset),
               est_q.homogeneous(offset));
    }
    sp->sc->est_p.x -= ref.x_end();
    offset += ctrl::Pose(ref.x_end(), 0, 0).rotate(offset.th);
    /* ending */
    sp->sc->disable();
    if (hw->mt->is_emergency())
      hw->mt->emergency_release(), hw->bz->play(hardware::Buzzer::EMERGENCY);
    hw->bz->play(hardware::Buzzer::CANCEL);
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
  void motor_test() {
    int value = sp->ui->waitForSelect(16);
    if (value < 0)
      return;
    if (!sp->ui->waitForCover(true))
      return;
    vTaskDelay(pdMS_TO_TICKS(500));
    float duty = (value < 8 ? value : value - 16) * 0.1f;
    hw->mt->drive(duty, duty);
    sp->ui->waitForCover();
    hw->mt->free();
    hw->bz->play(hardware::Buzzer::CANCEL);
  }
  void encoder_test() {
    int value = sp->ui->waitForSelect(16);
    hw->led->set(15);
    if (!sp->ui->waitForCover())
      return;
    vTaskDelay(pdMS_TO_TICKS(500));
    float pwm = 0.05f * value;
    hw->mt->drive(pwm, pwm);
    sp->ui->waitForCover(true);
    hw->mt->free();
  }
  void front_wall_attach_test() {
    while (1) {
      /* 開始待ち */
      if (!sp->ui->waitForCover(true))
        return;
      vTaskDelay(pdMS_TO_TICKS(500));
      sp->sc->enable();
      /* wall_attach start */
      bool result = false;
      hw->led->set(6);
      hw->tof->disable(); /*< ノイズ防止のためToFを無効化 */
      vTaskDelay(pdMS_TO_TICKS(20));
      sp->sc->est_p.clear();  //< 初動防止のため位置をクリア
      for (int i = 0; i < 2000; i++) {
        // if (is_break_state())
        //   break;
        /* 差分計算 */
        WheelParameter wp;
        for (int j = 0; j < 2; ++j) {
          wp.wheel[j] =
              sp->wd->distance_average.front[j] * model::front_wall_attach_gain;
        }
        wp.wheel2pole();
        /* 終了条件 */
        const float end = model::front_wall_attach_end;
        if (math_utils::sum_of_square(wp.wheel[0], wp.wheel[1]) < end) {
          result = true;  //< 補正成功
          break;
        }
        /* 制御 */
        const float sat_tra = 180.0f;  //< [mm/s]
        const float sat_rot = PI / 2;  //< [rad/s]
        sp->sc->set_target(math_utils::saturate(wp.tra, sat_tra),
                           math_utils::saturate(wp.rot, sat_rot));
        sp->sc->sampling_sync();
      }
      hw->bz->play(result ? hardware::Buzzer::SUCCESSFUL
                          : hardware::Buzzer::CANCEL);
      sp->sc->set_target(0, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
      hw->tof->enable();      //< ToF の有効化を忘れずに！
      sp->sc->est_p.clear();  //< 位置を補正
      hw->led->set(0);
      // ending
      sp->sc->disable();
      if (hw->mt->is_emergency()) {
        hw->bz->play(hardware::Buzzer::EMERGENCY);
        hw->mt->emergency_release();
      }
    }
  }
  void print_wall_detector() {
    // int value = sp->ui->waitForSelect(16);
    // if (value < 0)
    //   return;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
      LOGI(
          "Dist:[%5.1f %5.1f %5.1f %5.1f] Wall:[%c %c %c] "
          "ToF:[%3d mm %3d ms (%3d mm)]",
          (double)sp->wd->distance.side[0], (double)sp->wd->distance.front[0],
          (double)sp->wd->distance.front[1], (double)sp->wd->distance.side[1],
          sp->wd->is_wall[0] ? 'X' : '_', sp->wd->is_wall[2] ? 'X' : '_',
          sp->wd->is_wall[1] ? 'X' : '_', hw->tof->getDistance(),
          hw->tof->passedTimeMs(), hw->tof->getRangeRaw());
    }
  }
  void show_info() {
    /* show info */
    LOGI("I'm KERISE v%d.", KERISE_SELECT);
    LOGI("IDF version:  %s", esp_get_idf_version());
    LOGI("CPU Freq:     %u [MHz]", ets_get_cpu_frequency());
    peripheral::SPIFFS::show_info();
  }
  bool check_chip() {
    auto mac = peripheral::ESP::get_mac();
    if (mac != model::MAC_ID) {
      LOGW("MAC ID mismatched!");
      LOGW("MAC ID: 0x%012llX != 0x%012llX", mac, model::MAC_ID);
      return false;
    }
    return true;
  }

 public:
  Machine() {}
  bool init() {
    bool result = true;
    /* System */
    peripheral::SPIFFS::init() || (result = false);
    /* show info */
    show_info();
    /* check chip */
    if (!check_chip()) {
      auto* bz = hardware::Buzzer::get_instance();
      bz->init(BUZZER_PIN, BUZZER_LEDC_CHANNEL, BUZZER_LEDC_TIMER);
      bz->play(hardware::Buzzer::TIMEOUT);
      return false;
    }
    /* Hardware */
    hw = new hardware::Hardware();
    hw->init() || (result = false);
    /* Supporters */
    sp = new supporters::Supporters(hw);
    sp->init() || (result = false);
    /* Agents */
    ma = new MoveAction(hw, sp, model::TrajectoryTrackerGain);
    mr = new MazeRobot(hw, sp, ma);
    /* Others */
    lgr = new Logger();
    /* start tasks */
    task_drive.start(this, &Machine::drive, "Drive", 4096, 2, tskNO_AFFINITY);
    task_print.start(this, &Machine::print, "Print", 4096, 1, tskNO_AFFINITY);
    /* Ending */
    if (!result) {
      hw->bz->play(hardware::Buzzer::ERROR);
      return false;
    }
    return true;
  }
  void drive() {
    // driveAutomatically();
    while (1)
      driveManually();
    vTaskDelay(portMAX_DELAY);
  }
  void print() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(9));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(90));
      // hw->tof->print();
      // sp->wd->print();
      // hw->rfl->csv();
      // hw->enc->csv();
    }
  }

 private:
  freertospp::Task<Machine> task_drive;
  freertospp::Task<Machine> task_print;

  /* Hardware */
  hardware::Hardware* hw;
  /* Supporter */
  supporters::Supporters* sp;
  /* Agents */
  MoveAction* ma;
  MazeRobot* mr;
  /* Logger */
  Logger* lgr;
};

}  // namespace machine
