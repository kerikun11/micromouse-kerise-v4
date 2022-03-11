/**
 * @file move_action.h
 * @brief Move Action
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include "config/model.h"
#include "config/slalom_shapes.h"
#include "hardware/hardware.h"
#include "supporters/supporters.h"

#include <MazeLib/RobotBase.h> //< for RobotBase::SearchAction
#include <ctrl/accel_designer.h>
#include <ctrl/slalom.h>
#include <ctrl/straight.h>
#include <ctrl/trajectory_tracker.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <utils/concurrent_queue.hpp>
#include <utils/math_utils.hpp> //< for round2, saturate

#include <cmath>
#include <condition_variable>
#include <mutex>

#define DEBUG_WALL_ATTACH_EMERGENCY 0

class MoveAction {
public:
  /* Action Category */
  enum TaskAction : char {
    TaskActionSearchRun = 'S',
    TaskActionFastRun = 'F',
    TaskActionPositionRecovery = 'P',
  };
  /* Run Parameters */
  struct RunParameter {
  public:
    /* common flags */
    bool diag_enabled = 1;
    bool unknown_accel_enabled = 1;
    bool front_wall_fix_enabled = 1;
    bool side_wall_avoid_enabled = 1;
    bool side_wall_fix_theta_enabled = 1;
    bool side_wall_fix_v90_enabled = 1;
    bool side_wall_cut_enabled = 0;
    /* common values */
    float v_max = 720;
    float a_max = 3600;
    float j_max = 240'000;
    std::array<float, field::ShapeIndexMax> v_slalom;
    /* search run */
    float v_search = 330;
    float v_unknown_accel = 600;
    /* fast run */
    float fan_duty = 0.4f;

  public:
    // [1*1.05**i for i in range(0, 4)]: [1.0, 1.05, 1.1025, 1.1576]
    static constexpr float vs_factor = 1.05;
    // [int(720*1.2**i) for i in range(0, 4)]: [720, 864, 1036, 1244]
    static constexpr float vm_factor = 1.1;
    // [int(3600*1.05**i) for i in range(0, 4)]: [3600, 3780, 3969, 4167]
    static constexpr float am_factor = 1.05;

  public:
    RunParameter() {
      for (int i = 0; i < field::ShapeIndexMax; ++i)
        // v_slalom[i] = field::shapes[i].v_ref;
        v_slalom[i] = v_search;
    }
    void up(const int cnt = 1) {
      for (int i = 0; i < cnt; ++i) {
        for (auto &vs : v_slalom)
          vs *= vs_factor;
        v_max *= vm_factor, a_max *= am_factor;
      }
    }
    void down(const int cnt = 1) {
      for (int i = 0; i < cnt; ++i) {
        for (auto &vs : v_slalom)
          vs /= vs_factor;
        v_max /= vm_factor, a_max /= am_factor;
      }
    }
  };

public:
  RunParameter rp_search;
  RunParameter rp_fast;

private:
  hardware::Hardware *hw;
  supporters::Supporters *sp;

public:
  MoveAction(hardware::Hardware *hw, supporters::Supporters *sp,
             const ctrl::TrajectoryTracker::Gain &gain)
      : hw(hw), sp(sp), tt_gain(gain) {
    /* set default parameters */
    for (auto &vs : rp_search.v_slalom)
      vs = rp_search.v_search;
    for (auto &vs : rp_fast.v_slalom)
      vs = rp_search.v_search;
    /* デフォルトは既知区間斜めを無効化 */
    rp_search.diag_enabled = false;
    /* start */
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "MoveAction", 8192, this, 4, NULL);
  }

  void enable(const TaskAction ta) {
    // set parameter
    this->task_action = ta;
    // enable
    state_update(State::STATE_RUNNING);
  }
  void disable() {
    if (state != State::STATE_DISABLED)
      state_update(State::STATE_BREAKING);
    state_wait(State::STATE_DISABLED);
  }
  void waitForEndAction() { //
    state_wait(~State::STATE_RUNNING);
  }
  void enqueue_action(const MazeLib::RobotBase::SearchAction action) {
    sa_queue.push(action);
    if (state == STATE_WAITING)
      state_update(State::STATE_RUNNING);
  }
  void set_fast_path(const std::string &fast_path) {
    this->fast_path = fast_path;
    // if (state == STATE_WAITING)
    //   state_update(State::STATE_RUNNING);
  }
  void emergency_release() {
    if (hw->mt->is_emergency()) {
      hw->bz->play(hardware::Buzzer::EMERGENCY);
      sp->sc->disable();
      hw->fan->drive(0);
      vTaskDelay(pdMS_TO_TICKS(400));
      hw->mt->emergency_release();
      hw->tof->enable();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  const auto &getSensedWalls() const { return is_wall; }
  void calibration() {
    sp->sc->disable();
    hw->bz->play(hardware::Buzzer::CALIBRATION);
    hw->imu->calibration();
    hw->enc->clear_offset();
  }
  void set_unknown_accel_flag(bool flag) {
    continue_straight_if_no_front_wall = flag;
  }

private:
  ctrl::TrajectoryTracker::Gain tt_gain;
  TaskAction task_action = TaskActionSearchRun;
  ctrl::Pose offset;
  std::array<bool, 3> is_wall;
  bool continue_straight_if_no_front_wall = false;
  typedef struct {
    bool prev_wall[2];
    float prev_x[2];
  } wall_cut_data_t;

  /* State Manager */
  enum State : uint8_t {
    STATE_DISABLED = 1 << 0, //< 停止中
    STATE_RUNNING = 1 << 1,  //< キューの内容を走行中
    STATE_WAITING = 1 << 2,  //< キューが空になって待機中
    STATE_BREAKING = 1 << 3, //< 離脱中
  };
  enum State state = State::STATE_DISABLED;
  std::mutex state_mutex;
  std::condition_variable state_cv;
  void state_update(enum State state_new) {
    std::lock_guard<std::mutex> lock_guard(state_mutex);
    state = state_new;
    state_cv.notify_all();
  }
  void state_wait(uint8_t state_mask) {
    std::unique_lock<std::mutex> unique_lock(state_mutex);
    state_cv.wait(unique_lock, [&] { return state & state_mask; });
  }
  bool is_break_state() const {
    return hw->mt->is_emergency() || state == State::STATE_BREAKING;
  }

  void task() {
    while (1) {
      /* wait for action started */
      state_wait(State::STATE_RUNNING);
      /* start action */
      switch (task_action) {
      case TaskAction::TaskActionSearchRun:
        search_run_task();
        break;
      case TaskAction::TaskActionFastRun:
        fast_run_task(fast_path);
        break;
      case TaskAction::TaskActionPositionRecovery:
        position_recovery();
        break;
      default:
        break;
      }
      /* end action */
      sp->sc->disable();
      state_update(State::STATE_DISABLED);
    }
  }
  bool isAlong() const {
    return int(std::abs(offset.th) * 180 / PI + 1) % 90 < 2;
  }
  bool isDiag() const {
    return int(std::abs(offset.th) * 180 / PI + 45 + 1) % 90 < 2;
  }

  bool front_wall_attach(bool force = false) {
    if (is_break_state())
      return false;
    // 強制モードのときは1マス先の壁でも補正
    if (force && hw->tof->getDistance() > field::SegWidthFull * 4 / 3)
      return false;
    // 通常モードのときは1マス以内
    if (!force && hw->tof->getDistance() > field::SegWidthFull)
      return false;
#if DEBUG_WALL_ATTACH_EMERGENCY
    if (hw->mt->is_emergency())
      hw->led->set(10), vTaskDelay(portMAX_DELAY); //< for debug
#endif
    /* wall_attach start */
    bool result = false;
    hw->led->set(6);
    hw->tof->disable(); /*< ノイズ防止のためToFを無効化 */
    vTaskDelay(pdMS_TO_TICKS(20));
    sp->sc->reset(); //< 初動防止のため位置をクリア
    for (int i = 0; i < 2000; i++) {
#if DEBUG_WALL_ATTACH_EMERGENCY
      if (hw->mt->is_emergency())
        hw->led->set(11), vTaskDelay(portMAX_DELAY); //< for debug
#endif
      if (is_break_state())
        break;
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
        result = true; //< 補正成功
        break;
      }
      /* 制御 */
      const float sat_tra = 180.0f; //< [mm/s]
      const float sat_rot = PI;     //< [rad/s]
      sp->sc->set_target(math_utils::saturate(wp.tra, sat_tra),
                         math_utils::saturate(wp.rot, sat_rot));
      sp->sc->sampling_sync();
    }
    hw->bz->play(result ? hardware::Buzzer::SUCCESSFUL
                        : hardware::Buzzer::CANCEL);
    sp->sc->set_target(0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    hw->tof->enable(); //< ToF の有効化を忘れずに！
    sp->sc->reset();   //< 位置を補正
    hw->led->set(0);
#if DEBUG_WALL_ATTACH_EMERGENCY
    if (hw->mt->is_emergency())
      hw->led->set(12), vTaskDelay(portMAX_DELAY); //< for debug
#endif
#if 0
    /* 謎のバグ回避 */
    if (hw->mt->is_emergency()) {
      emergency_release();
      sp->sc->enable();
      return false;
    }
#endif
    return result;
  }
  void front_wall_fix(const RunParameter &rp, bool force = false) {
    /* 適用条件の判定 */
    if (!rp.front_wall_fix_enabled || !hw->tof->isValid())
      return;
    const int tof_mm = hw->tof->getDistance();
    const int passed_ms = hw->tof->passedTimeMs();
    /* 前壁1マス付近、かつ、サンプリング後1ms以内の場合のみ */
    if (!force && (std::abs(tof_mm - 90) > 30 || passed_ms > 1))
      return;
    if (force && (tof_mm < 60 || 150 < tof_mm || passed_ms > 6))
      return;
    /* 現在の姿勢が区画に対して垂直か調べる */
    const auto &p = sp->sc->est_p;       //< 局所座標系における位置
    const float th_g = offset.th + p.th; //< グローバル姿勢
    const float th_g_w = math_utils::round2(th_g, PI / 2); //< 直近の壁の姿勢
    constexpr float theta_threshold = PI * 0.5f / 180;
    if (std::abs(th_g - th_g_w) > theta_threshold)
      return;
    /* 壁との距離を取得 */
    constexpr float wall_fix_offset = model::wall_fix_offset; //< 大: 壁に近く
    const float d_tof =
        wall_fix_offset + tof_mm - passed_ms * 1e-3f * sp->sc->ref_v.tra;
    /* グローバル位置に変換 */
    const auto p_g = offset + p.rotate(offset.th); //< グローバル位置
    const float d_tof_g = p_g.rotate(-p_g.th).x + d_tof; //< 壁距離(グローバル)
    /* 基準となる前壁距離 (グローバル) */
    const float d_ref_g = math_utils::round2(d_tof_g, field::SegWidthFull);
    /* 前壁の距離誤差 */
    const float x_diff = d_ref_g - d_tof_g;
    /* 壁の有無の判断 */
    const float x_diff_abs = std::abs(x_diff);
    if (x_diff_abs > 30) //< [mm]
      return;
    /* 局所位置の修正 */
    const auto p_fix = ctrl::Pose(x_diff).rotate(p.th); //< ローカル座標に変換
    const float alpha = force ? 0.4f : 0.1f; //< 補正割合 (0: 補正なし)
    sp->sc->fix_pose({alpha * p_fix.x, alpha * p_fix.y, 0}, force);
    /* お知らせ */
#if 0
    if (x_diff_abs < 5) {
      hw->bz->play(hardware::Buzzer::SHORT7);
    } else {
      hw->bz->play(hardware::Buzzer::SHORT6);
    }
#endif
    return;
  }
  void side_wall_fix_v90(const RunParameter &rp) {
    if (!rp.side_wall_fix_v90_enabled) {
      return;
    }
    /* 現在の姿勢が区画に対して垂直か調べる */
    const auto &p = sp->sc->est_p;       //< 局所座標系における位置
    const float th_g = offset.th + p.th; //< グローバル姿勢
    const float th_g_w = math_utils::round2(th_g, PI / 2); //< 直近の壁の姿勢
    constexpr float theta_threshold = PI * 0.5f / 180;
    if (std::abs(th_g - th_g_w) > theta_threshold)
      return;
    /* グローバル位置に変換 */
    const auto p_g = offset + p.rotate(offset.th); //< グローバル位置
    const float y_g = p_g.rotate(-th_g_w).y;       //< グローバル横位置
    const float y_w = math_utils::round2(y_g, field::SegWidthFull); //< 内側
    const float y_in = std::abs(y_g - y_w); //< 内側の柱との距離 (30mm 基準)
    /* 壁との距離を取得 */
    for (int i = 0; i < 2; i++) {
      const float y_out = 45 + sp->wd->distance.side[i]; //< 外壁との距離
      const float y_diff = y_in + y_out - 90;
      const float y_diff_abs = std::abs(y_diff);
      const float alpha = 0.1f;
      if (y_diff_abs < 20) {
        if (i == 0) {
          sp->sc->fix_pose(ctrl::Pose(0, alpha * y_diff).rotate(p.th));
          hw->led->set(hw->led->get() | 1);
        } else {
          sp->sc->fix_pose(ctrl::Pose(0, alpha * y_diff).rotate(p.th));
          hw->led->set(hw->led->get() | 8);
        }
      }
    }
  }
  void side_wall_avoid(const RunParameter &rp, const float remain) {
    /* 有効 かつ 一定速度より大きい かつ 姿勢が整っているときのみ */
    constexpr float theta_threshold = PI * 0.5f / 180;
    if (!rp.side_wall_avoid_enabled || sp->sc->est_v.tra < 240.0f ||
        std::abs(sp->sc->est_p.th) > theta_threshold) {
      return;
    }
    uint8_t led_flags = hw->led->get();
    /* 壁と平行 */
    if (isAlong()) {
      const float alpha = model::wall_avoid_alpha; //< 補正割合 (0: 補正なし)
      const float wall_dist_thr = 10; //< 遠方の閾値（近接は閾値なし）
      float y_error = 0;              //< 姿勢の補正用変数
      if (sp->wd->distance.side[0] < wall_dist_thr) {
        const float y_fix = -sp->wd->distance.side[0] - sp->sc->est_p.y;
        y_error += y_fix;
        sp->sc->fix_pose({0, alpha * y_fix});
        led_flags |= 8;
      }
      if (sp->wd->distance.side[1] < wall_dist_thr) {
        const float y_fix = sp->wd->distance.side[1] - sp->sc->est_p.y;
        y_error += y_fix;
        sp->sc->fix_pose({0, alpha * y_fix});
        led_flags |= 1;
      }
      /* 機体姿勢の補正 (壁に寄り続けている→姿勢を補正) */
      if (rp.side_wall_fix_theta_enabled)
        sp->sc->fix_pose({0, 0, y_error * model::wall_fix_theta_gain});
#if 0
      /* 櫛の壁制御 (KERISE v5) */
      if (hw->tof->getDistance() > field::SegWidthFull * 3 / 2) {
        const float comb_threashold = model::wall_comb_threshold;
        const float comb_shift = 0.1f;
        if (sp->wd->distance.front[0] < comb_threashold) {
          sp->sc->est_p.y += comb_shift;
          led_flags |= 4;
          hw->bz->play(hardware::Buzzer::SHORT7);
        }
        if (sp->wd->distance.front[1] < comb_threashold) {
          sp->sc->est_p.y -= comb_shift;
          led_flags |= 2;
          hw->bz->play(hardware::Buzzer::SHORT7);
        }
      }
#endif
    }
#if 1
    /* 斜めの壁制御 */
    if (isDiag() && remain > field::SegWidthFull / 3) {
      const float alpha = 0.1;         //< 補正割合 (0: 補正なし)
      const float wall_dist_ref = -12; //< 大きく：補正強く
      if (sp->wd->distance.side[0] < wall_dist_ref) {
        sp->sc->fix_pose(
            ctrl::Pose(0, +alpha * (wall_dist_ref - sp->wd->distance.side[0])));
        led_flags |= 8;
      }
      if (sp->wd->distance.side[1] < wall_dist_ref) {
        sp->sc->fix_pose(
            ctrl::Pose(0, -alpha * (wall_dist_ref - sp->wd->distance.side[1])));
        led_flags |= 1;
      }
    }
#endif
    hw->led->set(led_flags);
  }
  void side_wall_cut(const RunParameter &rp, wall_cut_data_t &wall_cut_data) {
    if (!rp.side_wall_cut_enabled || isDiag())
      return;
    /* 左右それぞれ */
    for (int i = 0; i < 2; i++) {
      bool wall = sp->wd->is_wall[i];
      const float x = sp->sc->est_p.x;
      /* 壁の変化 */
      if (wall_cut_data.prev_wall[i] != wall) {
        if (wall) {
          wall_cut_data.prev_x[i] = x;
        } else {
          // float wall_width = x - wall_cut_data.prev_x[i];
          // if (15 < wall_width && wall_width < 60) {
          //   const float comb_shift = (wall_width - 15) * 0.5f;
          //   sp->sc->fix_pose({0, i == 0 ? comb_shift : -comb_shift});
          //   hw->bz->play(hardware::Buzzer::SHORT7);
          // }
        }
        // /* 姿勢が整っているときのみ */
        // constexpr float theta_threshold = PI * 0.5f / 180;
        // if (std::abs(sp->sc->est_p.th) < theta_threshold) {
        // const float wall_cut_offset = -15; /*< 大きいほど前へ */
        // const float x_abs = offset.rotate(offset.th).x + sp->sc->est_p.x;
        // const float x_abs_cut =
        //     math_utils::round2(x_abs, field::SegWidthFull);
        // const float fixed_x = x_abs_cut - x_abs + wall_cut_offset;
        // const float fixed_x_abs = std::abs(fixed_x);
        // const float alpha = 0.1f;
        // sp->sc->fix_pose(ctrl::Pose(alpha * fixed_x, 0, 0));
        // hw->bz->play(hardware::Buzzer::CANCEL);
        // }
      }
      wall_cut_data.prev_wall[i] = wall;
    }
  }
  void straight_x(const float distance, float v_max, float v_end,
                  const RunParameter &rp, bool unknown_accel = false) {
    if (is_break_state())
      return;
    /* 未知区間加速の反映 */
    v_end = unknown_accel ? rp.v_unknown_accel : v_end;
    v_max = unknown_accel ? rp.v_unknown_accel : v_max;
    /* 壁切れ用 */
    wall_cut_data_t wall_cut_data = {
        .prev_wall = {sp->wd->is_wall[0], sp->wd->is_wall[1]},
        .prev_x = {sp->sc->est_p.x, sp->sc->est_p.x},
    };
    /* 前壁補正 */
    front_wall_fix(rp, true); //< ステップ変化を許容
    /* 移動分が存在する場合 */
    if (distance - sp->sc->est_p.x > 0) {
      const float v_start = sp->sc->ref_v.tra;
      ctrl::TrajectoryTracker tt{tt_gain};
      ctrl::State ref_s;
      ctrl::straight::Trajectory trajectory;
      /* start */
      trajectory.reset(rp.j_max, rp.a_max, v_max, v_start,
                       std::max(v_end, 30.0f), distance - sp->sc->est_p.x,
                       sp->sc->est_p.x);
      tt.reset(v_start);
#if DEBUG_WALL_ATTACH_EMERGENCY
      auto lgr = new Logger();
      lgr->init({
          "ref_v.tra", "est_v.tra", "ref_a.tra", "est_a.tra", "ff.tra",
          "fbp.tra",   "fbi.tra",   "fbd.tra",   "ref_v.rot", "est_v.rot",
          "ref_a.rot", "est_a.rot", "ff.rot",    "fbp.rot",   "fbi.rot",
          "fbd.rot",   "ref_q.x",   "est_q.x",   "ref_q.y",   "est_q.y",
          "ref_q.th",  "est_q.th",
      });
#endif
      for (float t = 0; true; t += sp->sc->Ts) {
        if (is_break_state())
          break;
        /* 終了条件 */
        const float remain = distance - sp->sc->est_p.x;
        if (remain < 0 || t > trajectory.t_end())
          break; //< 静止の場合を考慮した条件
        /* 前壁制御 */
        if (isAlong() && hw->tof->isValid()) {
          const float tof_mm = hw->tof->getLog().average(2);
          /* 衝突被害軽減ブレーキ (AEBS) */
          if (remain > field::SegWidthFull && tof_mm < field::SegWidthFull)
            wall_stop_aebs();
          if (v_end > 1 && tof_mm < field::SegWidthFull / 2)
            wall_stop_aebs();
          /* 未知区間加速の緊急キャンセル */
          if (unknown_accel && tof_mm < 1.8f * field::SegWidthFull) {
            unknown_accel = false;
            trajectory.reset(rp.j_max, rp.a_max, rp.v_search, sp->sc->ref_v.tra,
                             rp.v_search, remain, sp->sc->est_p.x, t);
            hw->bz->play(hardware::Buzzer::MAZE_BACKUP);
          }
        }
        /* 情報の更新 */
        sp->sc->sampling_sync();
        /* 壁補正 */
        hw->led->set(0);
        front_wall_fix(rp);
        side_wall_avoid(rp, remain);
        side_wall_cut(rp, wall_cut_data);
        /* 軌道追従 */
        trajectory.update(ref_s, t);
        const auto ref =
            tt.update(sp->sc->est_p, sp->sc->est_v, sp->sc->est_a, ref_s);
        sp->sc->set_target(ref.v, ref.w, ref.dv, ref.dw);
#if DEBUG_WALL_ATTACH_EMERGENCY
        const auto &bd = sp->sc->fbc.getBreakdown();
        const auto &ref_q = ref_s.q;
        const auto &est_q = sp->sc->est_p;
        lgr->push({
            sp->sc->ref_v.tra, sp->sc->est_v.tra, sp->sc->ref_a.tra,
            sp->sc->est_a.tra, bd.ff.tra,         bd.fbp.tra,
            bd.fbi.tra,        bd.fbd.tra,        sp->sc->ref_v.rot,
            sp->sc->est_v.rot, sp->sc->ref_a.rot, sp->sc->est_a.rot,
            bd.ff.rot,         bd.fbp.rot,        bd.fbi.rot,
            bd.fbd.rot,        ref_q.x,           est_q.x,
            ref_q.y,           est_q.y,           ref_q.th,
            est_q.th,
        });
#endif
      }
#if DEBUG_WALL_ATTACH_EMERGENCY
      if (hw->mt->is_emergency()) {
        while (1) {
          // app_logi << "ref_v: " << sp->sc->ref_v << std::endl;
          // app_logi << "ref_a: " << sp->sc->ref_a << std::endl;
          // app_logi << "est_p: " << sp->sc->est_p << std::endl;
          // app_logi << "est_v: " << sp->sc->est_v << std::endl;
          // app_logi << "est_a: " << sp->sc->est_a << std::endl;
          // app_logi << "fbc.u: " << sp->sc->fbc.getBreakdown().u << std::endl;
          // app_logi << "tt.xi: " << tt.xi << std::endl;
          lgr->print();
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
      }
#endif
    }
    if (v_end < 1) {
      sp->sc->set_target(0, 0);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    /* 移動した量だけ位置を更新 */
    sp->sc->fix_pose({-distance, 0, 0}, true);
    offset += ctrl::Pose(distance, 0, 0).rotate(offset.th);
  }
  void turn(const float angle) {
    if (is_break_state())
      return;
    const float dddth_max = 2400 * PI;
    const float ddth_max = 54 * PI;
    const float dth_max = 4 * PI;
    constexpr float back_gain = model::turn_back_gain;
    ctrl::AccelDesigner ad(dddth_max, ddth_max, dth_max, 0, 0, angle);
    for (float t = 0; t < ad.t_end(); t += sp->sc->Ts) {
      if (is_break_state())
        break;
      sp->sc->sampling_sync();
      const float delta = sp->sc->est_p.x * std::cos(-sp->sc->est_p.th) -
                          sp->sc->est_p.y * std::sin(-sp->sc->est_p.th);
      sp->sc->set_target(-delta * back_gain, ad.v(t), 0, ad.a(t));
    }
    /* 確実に目標角度に持っていく処理 */
    float int_error = 0;
    while (1) {
      if (is_break_state())
        break;
      sp->sc->sampling_sync();
      float delta = sp->sc->est_p.x * std::cos(-sp->sc->est_p.th) -
                    sp->sc->est_p.y * std::sin(-sp->sc->est_p.th);
      const float Kp = 10.0f;
      const float Ki = 10.0f;
      const float error = angle - sp->sc->est_p.th;
      int_error += error * sp->sc->Ts;
      sp->sc->set_target(-delta * back_gain, Kp * error + Ki * int_error);
      if (std::abs(Kp * error) + std::abs(Ki * int_error) < 0.1f * PI)
        break;
    }
    /* 移動した量だけ位置を更新 */
    const auto net = ctrl::Pose(0, 0, angle);
    sp->sc->update_pose((sp->sc->est_p - net).rotate(-net.th));
    offset += net.rotate(offset.th);
  }
  void trace(ctrl::slalom::Trajectory &trajectory, const RunParameter &rp) {
    if (is_break_state())
      return;
    /* 前壁補正 */
    front_wall_fix(rp, true); //< ステップ変化を許容
    /* prepare */
    const float Ts = sp->sc->Ts;
    const float velocity = sp->sc->ref_v.tra;
    ctrl::TrajectoryTracker tt(tt_gain);
    ctrl::State s;
    /* start */
    tt.reset(velocity);
    trajectory.reset(velocity);
    s.q.x = sp->sc->est_p.x; /*< 既に移動した分を反映 */
    if (std::abs(sp->sc->est_p.x) > 1)
      hw->bz->play(hardware::Buzzer::MAZE_BACKUP); //< 現在位置が進みすぎ警告
    for (float t = 0; t < trajectory.getTimeCurve(); t += Ts) {
      if (is_break_state())
        break;
      /* データの更新 */
      sp->sc->sampling_sync();
      /* 壁補正 */
      hw->led->set(0);
      front_wall_fix(rp);
      side_wall_avoid(rp, 0);
      if (std::abs(trajectory.getShape().v_ref -
                   field::shapes[field::ShapeIndex::FV90].v_ref) < 0.01f)
        side_wall_fix_v90(rp); /*< V90の横壁補正 */
      /* 軌道を更新 */
      trajectory.update(s, t, Ts);
      const auto ref =
          tt.update(sp->sc->est_p, sp->sc->est_v, sp->sc->est_a, s);
      sp->sc->set_target(ref.v, ref.w, ref.dv, ref.dw);
      /* ToDo: 打ち切り条件を追加！！！ */
    }
    sp->sc->set_target(velocity, 0);
    /* 移動した量だけ位置を更新 */
    const auto &net = trajectory.getShape().curve;
    sp->sc->update_pose((sp->sc->est_p - net).rotate(-net.th));
    offset += net.rotate(offset.th);
  }
  void SlalomProcess(const field::ShapeIndex si, const bool mirror_x,
                     const bool reverse, float &straight,
                     const RunParameter &rp) {
    if (is_break_state())
      return;
    const auto &shape = field::shapes[si];
    ctrl::slalom::Trajectory st(shape, mirror_x);
    const auto straight_prev = shape.straight_prev;
    const auto straight_post = shape.straight_post;
    const float velocity = rp.v_slalom[si];
    straight += !reverse ? straight_prev : straight_post;
    /* ターン前の直線を消化 */
    if (straight > 0.1f) {
      /* ToDo: ちょっと手前で終えて前壁補正を行う */
      straight_x(straight, rp.v_max, velocity, rp);
      straight = 0;
    }
    /* スラローム */
    trace(st, rp);
    straight += reverse ? straight_prev : straight_post;
  }
  void u_turn() {
    if (is_break_state())
      return;
    /* 前壁あり：横壁なし方向でターン */
    /* 前壁なし：横壁あり方向でターン */
    int dir = (sp->wd->distance.side[0] > sp->wd->distance.side[1]) ? -1 : 1;
    dir = sp->wd->is_wall[2] ? -dir : dir;
    if (front_wall_attach()) {
      turn(dir * PI);
    } else {
      turn(dir * PI / 2);
      front_wall_attach();
      turn(dir * PI / 2);
    }
    /* 謎バグ: なぜか前進しているので修正 */
    sp->sc->fix_pose({model::CenterOffsetY, 0, 0}, true);
  }
  void wall_stop_aebs() {
    if (is_break_state())
      return;
    hw->bz->play(hardware::Buzzer::AEBS);
    // ToDo: compiler bug avoidance!
    for (float v = sp->sc->ref_v.tra; v > 0; v -= 12) {
      sp->sc->sampling_sync();
      sp->sc->set_target(v, 0);
    }
    sp->sc->set_target(0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    sp->sc->disable();
    hw->mt->emergency_stop();
    state_update(State::STATE_BREAKING);
  }
  void start_step(const RunParameter &rp) {
    if (is_break_state())
      return;
    sp->sc->disable();
    hw->mt->drive(-0.2f, -0.2f); /*< 背中を確実に壁につける */
    vTaskDelay(pdMS_TO_TICKS(500));
    hw->mt->free();
    sp->sc->sampling_sync();
    sp->sc->enable(); //< this resets est_p
    sp->sc->update_pose({model::TailLength + field::WallThickness / 2});
    offset = ctrl::Pose(field::SegWidthFull / 2, 0, PI / 2);
    straight_x(field::SegWidthFull, rp.v_search, rp.v_search, rp);
  }
  void start_init() {
    if (is_break_state())
      return;
    front_wall_attach();
    turn(PI / 2);
    front_wall_attach();
    turn(PI / 2);
    put_back();
    sp->sc->disable();
    state_update(State::STATE_BREAKING);
  }
  void put_back() {
    if (is_break_state())
      return;
    const int max_v = 150;
    const float th_gain = 50.0f;
    for (int i = 0; i < max_v; i++) {
      sp->sc->set_target(-i, -sp->sc->est_p.th * th_gain);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 100; i++) {
      sp->sc->set_target(-max_v, -sp->sc->est_p.th * th_gain);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    sp->sc->disable();
    hw->mt->drive(-0.2f, -0.2f); /*< 背中を確実に壁につける */
    vTaskDelay(pdMS_TO_TICKS(400));
    hw->mt->drive(0, 0);
  }

private:
  utils::concurrent_queue<MazeLib::RobotBase::SearchAction> sa_queue;

  void search_run_task() {
    const auto &rp = rp_search;
#if DEBUG_WALL_ATTACH_EMERGENCY
    if (hw->mt->is_emergency())
      hw->led->set(1), vTaskDelay(portMAX_DELAY); //< for debug
#endif
    /* スタート */
    // sp->sc->reset();
    // vTaskDelay(pdMS_TO_TICKS(100)); //< 緊急ループ防止の delay
    sp->sc->enable();
    /* とりあえず区画の中心に配置 */
    offset = ctrl::Pose(field::SegWidthFull / 2, field::SegWidthFull / 2);
    while (1) {
#if DEBUG_WALL_ATTACH_EMERGENCY
      if (hw->mt->is_emergency())
        hw->led->set(2), vTaskDelay(portMAX_DELAY); //< for debug
#endif
      /* 離脱確認 */
      if (is_break_state())
        break;
      /* 壁を確認 */
      is_wall = sp->wd->is_wall;
      /* 探索器に終了を通知 */
      if (sa_queue.empty())
        state_update(State::STATE_WAITING);
      /* Actionがキューされるまで直進で待つ */
      if (sa_queue.empty())
        search_run_queue_wait_decel(rp);
#if DEBUG_WALL_ATTACH_EMERGENCY
      if (hw->mt->is_emergency())
        hw->led->set(3), vTaskDelay(portMAX_DELAY); //< for debug
#endif
      /* 既知区間走行 */
      if (sa_queue.size() >= 2)
        search_run_known(rp);
#if DEBUG_WALL_ATTACH_EMERGENCY
      if (hw->mt->is_emergency())
        hw->led->set(4), vTaskDelay(portMAX_DELAY); //< for debug
#endif
      /* 探索走行 */
      if (!sa_queue.empty()) {
        const auto action = sa_queue.front();
        sa_queue.pop();
        search_run_switch(action, rp);
#if DEBUG_WALL_ATTACH_EMERGENCY
        if (hw->mt->is_emergency())
          hw->led->set(5), vTaskDelay(portMAX_DELAY); //< for debug
#endif
      }
    }
    // cleaning
    while (!sa_queue.empty())
      sa_queue.pop();
    sp->sc->disable();
#if DEBUG_WALL_ATTACH_EMERGENCY
    if (hw->mt->is_emergency())
      hw->led->set(8), vTaskDelay(portMAX_DELAY); //< for debug
#endif
  }
  void search_run_queue_wait_decel(const RunParameter &rp) {
    /* Actionがキューされるまで減速しながら待つ */
    ctrl::TrajectoryTracker tt(tt_gain);
    ctrl::State ref_s;
    const auto v_start = sp->sc->ref_v.tra;
    const float x_start = sp->sc->est_p.x;
    ctrl::AccelCurve ac(rp.j_max, rp.a_max, v_start, 0); //< なめらかに減速
    /* start */
    tt.reset(v_start);
    for (float t = 0; sa_queue.empty(); t += sp->sc->Ts) {
      if (is_break_state())
        break;
      sp->sc->sampling_sync();
      /* 壁補正 */
      hw->led->set(0);
      front_wall_fix(rp);
      side_wall_avoid(rp, 0);
      const auto ref =
          tt.update(sp->sc->est_p, sp->sc->est_v, sp->sc->est_a,
                    ctrl::Pose(ac.x(t) + x_start), ctrl::Pose(ac.v(t)),
                    ctrl::Pose(ac.a(t)), ctrl::Pose(ac.j(t)));
      sp->sc->set_target(ref.v, ref.w, ref.dv, ref.dw);
    }
    /* 注意: 現在位置はやや前に進んだ状態 */
#if DEBUG_WALL_ATTACH_EMERGENCY
    if (hw->mt->is_emergency())
      hw->led->set(13), vTaskDelay(portMAX_DELAY); //< for debug
#endif
  }
  void search_run_known(const RunParameter &rp) {
    /* path の作成 */
    std::string path;
    while (1) {
      if (sa_queue.empty())
        break;
      const auto action = sa_queue.front();
      if (action == MazeLib::RobotBase::SearchAction::ST_HALF ||
          action == MazeLib::RobotBase::SearchAction::ST_FULL ||
          action == MazeLib::RobotBase::SearchAction::TURN_L ||
          action == MazeLib::RobotBase::SearchAction::TURN_R) {
        path += action;
        sa_queue.pop();
      } else {
        break;
      }
    }
    /* 既知区間走行 */
    if (path.size()) {
      /* 既知区間パターンに変換 */
      path =
          MazeLib::RobotBase::pathConvertSearchToKnown(path, rp.diag_enabled);
      /* 既知区間走行 */
      float straight = 0;
      for (int path_index = 0; path_index < path.length(); path_index++) {
        const auto action =
            static_cast<MazeLib::RobotBase::FastAction>(path[path_index]);
        fast_run_switch(action, straight, rp);
      }
#if DEBUG_WALL_ATTACH_EMERGENCY
      if (hw->mt->is_emergency())
        hw->led->set(14), vTaskDelay(portMAX_DELAY); //< for debug
#endif
      /* 最後の直線を消化 */
      if (straight > 0.1f) {
        straight_x(straight, rp.v_max, rp.v_search, rp);
        straight = 0;
      }
#if DEBUG_WALL_ATTACH_EMERGENCY
      if (hw->mt->is_emergency())
        hw->led->set(15), vTaskDelay(portMAX_DELAY); //< for debug
#endif
    }
  }
  void search_run_switch(const MazeLib::RobotBase::SearchAction action,
                         const RunParameter &rp) {
    if (is_break_state())
      return;
    const bool no_front_front_wall =
        hw->tof->getDistance() >
        field::SegWidthFull * 2 + field::SegWidthFull / 2;
    const bool unknown_accel = rp.unknown_accel_enabled &&
                               continue_straight_if_no_front_wall &&
                               no_front_front_wall;
    const float v_s = rp.v_search;
    switch (action) {
    case MazeLib::RobotBase::SearchAction::START_STEP:
      start_step(rp);
      break;
    case MazeLib::RobotBase::SearchAction::START_INIT:
      start_init();
      break;
    case MazeLib::RobotBase::SearchAction::ST_FULL:
      if (hw->tof->getDistance() < field::SegWidthFull)
        return wall_stop_aebs();
      straight_x(field::SegWidthFull, v_s, v_s, rp, unknown_accel);
      break;
    case MazeLib::RobotBase::SearchAction::ST_HALF:
      straight_x(field::SegWidthFull / 2, v_s, v_s, rp);
#if DEBUG_WALL_ATTACH_EMERGENCY
      if (hw->mt->is_emergency())
        hw->led->set(7), vTaskDelay(portMAX_DELAY); //< for debug
#endif
      break;
    case MazeLib::RobotBase::SearchAction::TURN_L:
      if (sp->sc->est_p.x > 5.0f || sp->sc->ref_v.tra > v_s * 1.2f ||
          (sp->wd->is_wall[2] &&
           std::abs(hw->tof->getDistance() - field::SegWidthFull) > 20)) {
        straight_x(field::SegWidthFull / 2, v_s, 0, rp);
        front_wall_attach();
        turn(PI / 2);
        straight_x(field::SegWidthFull / 2, v_s, v_s, rp);
      } else {
        static ctrl::slalom::Trajectory st(
            field::shapes[field::ShapeIndex::S90], 0);
        straight_x(st.getShape().straight_prev, v_s, v_s, rp);
        if (sp->wd->is_wall[0])
          return wall_stop_aebs();
        trace(st, rp);
        straight_x(st.getShape().straight_post, v_s, v_s, rp);
      }
      break;
    case MazeLib::RobotBase::SearchAction::TURN_R:
      if (sp->sc->est_p.x > 5.0f || sp->sc->ref_v.tra > v_s * 1.2f ||
          (sp->wd->is_wall[2] &&
           std::abs(hw->tof->getDistance() - field::SegWidthFull) > 20)) {
        straight_x(field::SegWidthFull / 2, v_s, 0, rp);
        front_wall_attach();
        turn(-PI / 2);
        straight_x(field::SegWidthFull / 2, v_s, v_s, rp);
      } else {
        static ctrl::slalom::Trajectory st(
            field::shapes[field::ShapeIndex::S90], 1);
        straight_x(st.getShape().straight_prev, v_s, v_s, rp);
        if (sp->wd->is_wall[1])
          return wall_stop_aebs();
        trace(st, rp);
        straight_x(st.getShape().straight_post, v_s, v_s, rp);
      }
      break;
    case MazeLib::RobotBase::SearchAction::ROTATE_180:
      u_turn();
#if DEBUG_WALL_ATTACH_EMERGENCY
      if (hw->mt->is_emergency())
        hw->led->set(6), vTaskDelay(portMAX_DELAY); //< for debug
#endif
      break;
    case MazeLib::RobotBase::SearchAction::ST_HALF_STOP:
      straight_x(field::SegWidthFull / 2, v_s, 0, rp);
      break;
    }
  }

private:
  std::string fast_path;

  bool fast_run_task(const std::string &search_actions) {
    /* 走行パラメータを取得 */
    const auto &rp = rp_fast;
    /* 最短走行用にパターンを置換 */
    const auto path = MazeLib::RobotBase::pathConvertSearchToFast(
        search_actions, rp.diag_enabled);
    /* キャリブレーション */
    calibration();
    /* 壁に背中を確実につける */
    hw->mt->drive(-0.25f, -0.25f);
    vTaskDelay(pdMS_TO_TICKS(200));
    hw->mt->free();
    /* ファンを始動 */
    hw->fan->drive(rp.fan_duty);
    vTaskDelay(pdMS_TO_TICKS(400)); //< ファンの回転数が一定なるのを待つ
    /* 走行開始 */
    sp->sc->enable();
    /* 初期位置を設定 */
    offset = ctrl::Pose(field::SegWidthFull / 2,
                        model::TailLength + field::WallThickness / 2, PI / 2);
    /* 最初の直線を追加 */
    float straight =
        field::SegWidthFull / 2 - model::TailLength - field::WallThickness / 2;
    /* 走行 */
    for (int path_index = 0; path_index < path.length(); path_index++) {
      if (is_break_state())
        break;
      const auto action =
          static_cast<MazeLib::RobotBase::FastAction>(path[path_index]);
      fast_run_switch(action, straight, rp);
    }
    /* 最後の直線を消化 */
    if (straight > 0.1f) {
      straight_x(straight, rp.v_max, 0, rp);
      straight = 0;
    }
    /* 停止処理 */
    sp->sc->set_target(0, 0);
    hw->fan->drive(0);
    /* クラッシュでない場合は機体が完全に停止するまで待つ */
    if (!hw->mt->is_emergency()) {
      vTaskDelay(pdMS_TO_TICKS(400));
      hw->bz->play(hardware::Buzzer::COMPLETE);
    }
    sp->sc->disable();
    return true;
  }
  void fast_run_switch(const MazeLib::RobotBase::FastAction action,
                       float &straight, const RunParameter &rp) {
    switch (action) {
    case MazeLib::RobotBase::FastAction::F45_L:
      SlalomProcess(field::ShapeIndex::F45, 0, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F45_R:
      SlalomProcess(field::ShapeIndex::F45, 1, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F45_LP:
      SlalomProcess(field::ShapeIndex::F45, 0, 1, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F45_RP:
      SlalomProcess(field::ShapeIndex::F45, 1, 1, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::FV90_L:
      SlalomProcess(field::ShapeIndex::FV90, 0, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::FV90_R:
      SlalomProcess(field::ShapeIndex::FV90, 1, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::FS90_L:
      SlalomProcess(field::ShapeIndex::FS90, 0, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::FS90_R:
      SlalomProcess(field::ShapeIndex::FS90, 1, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F90_L:
      SlalomProcess(field::ShapeIndex::F90, 0, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F90_R:
      SlalomProcess(field::ShapeIndex::F90, 1, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F135_L:
      SlalomProcess(field::ShapeIndex::F135, 0, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F135_R:
      SlalomProcess(field::ShapeIndex::F135, 1, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F135_LP:
      SlalomProcess(field::ShapeIndex::F135, 0, 1, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F135_RP:
      SlalomProcess(field::ShapeIndex::F135, 1, 1, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F180_L:
      SlalomProcess(field::ShapeIndex::F180, 0, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F180_R:
      SlalomProcess(field::ShapeIndex::F180, 1, 0, straight, rp);
      break;
    case MazeLib::RobotBase::FastAction::F_ST_FULL:
      straight += field::SegWidthFull;
      break;
    case MazeLib::RobotBase::FastAction::F_ST_HALF:
      straight += field::SegWidthFull / 2;
      break;
    case MazeLib::RobotBase::FastAction::F_ST_DIAG:
      straight += field::SegWidthDiag / 2;
      break;
    default:
      break;
    }
  }

private:
  bool position_recovery() {
    /* 1周回って壁を探す */
    static constexpr float dddth_max = 4800 * PI;
    static constexpr float ddth_max = 48 * PI;
    static constexpr float dth_max = 2 * PI;
    const float angle = 2 * PI;
    ctrl::AccelDesigner ad(dddth_max, ddth_max, dth_max, 0, 0, angle);
    /* 走査データ格納用変数 */
    constexpr int table_size = 96;
    std::bitset<table_size> is_valid;
    std::array<uint16_t, table_size> table;
    table.fill(255);
    /* start */
    sp->sc->enable();
    int index = 0;
    for (float t = 0; t < ad.t_end(); t += sp->sc->Ts) {
      if (is_break_state())
        break;
      sp->sc->sampling_sync();
      const float delta = sp->sc->est_p.x * std::cos(-sp->sc->est_p.th) -
                          sp->sc->est_p.y * std::sin(-sp->sc->est_p.th);
      constexpr float back_gain = model::turn_back_gain;
      sp->sc->set_target(-delta * back_gain, ad.v(t), 0, ad.a(t));
      if (ad.x(t) > 2 * PI * index / table_size) {
        index++;
        table[index % table_size] = hw->tof->getDistance();
        is_valid[index % table_size] = hw->tof->isValid();
      }
    }
    /* 最小分散を探す */
    float min_var = 999;
    int min_i = 0;
    for (int i = 0; i < table_size; ++i) {
      if (!is_valid[i])
        continue;
      const int window = table_size / 12;
      int sum = 0;
      for (int j = -window / 2; j < window / 2; ++j)
        sum += table[(table_size + i + j) % table_size];
      int ave = sum / window;
      int var = 0;
      for (int j = -window / 2; j < window / 2; ++j)
        var = std::pow(table[(table_size + i + j) % table_size] - ave, 2);
      if (min_var > var) {
        min_var = var;
        min_i = i;
      }
    }
    /* 最小分散の方向を向く */
    sp->sc->reset();
    turn(2 * PI * min_i / table_size);
    /* 壁が遠い場合は直進する */
    if (hw->tof->isValid() && hw->tof->getDistance() > field::SegWidthFull / 2)
      straight_x(hw->tof->getDistance() - field::SegWidthFull / 2, 240, 0,
                 rp_search);
    /* 前壁補正 */
    front_wall_attach(true);
    /* 前壁補正のため、壁がありそうな方に回転 */
    turn(sp->wd->distance.side[0] < sp->wd->distance.side[1] ? PI / 2
                                                             : -PI / 2);
    /* 壁のない方向を向く */
    while (!hw->mt->is_emergency()) {
      if (hw->tof->getDistance() > field::SegWidthFull)
        break;
      front_wall_attach(true);
      turn(-PI / 2);
    }
    sp->sc->disable();
    sp->sc->reset();
    // if (hw->mt->is_emergency()) {
    //   hw->bz->play(hardware::Buzzer::EMERGENCY);
    //   hw->mt->emergency_release();
    //   return false;
    // }
    return true;
  }
};
