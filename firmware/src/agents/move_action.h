/**
 * @file move_action.h
 * @brief Move Action
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include "config/model.h"
#include "config/slalom_shapes.h"
#include "machine/global.h"

#include <MazeLib/RobotBase.h> //< for RobotBase::SearchAction
#include <ctrl/accel_designer.h>
#include <ctrl/slalom.h>
#include <ctrl/straight.h>
#include <ctrl/trajectory_tracker.h>
#include <utils/concurrent_queue.hpp>
#include <utils/math_utils.hpp>

#include <cmath>

#if 0
#define ma_log app_logi
#else
#define ma_log std::ostream(0)
#endif

class MoveAction {
public:
  static constexpr float v_unknown_accel = 600;
  static constexpr float v_search = 330;

public:
  struct RunParameter {
  public:
    bool diag_enabled = 1;
    bool unknown_accel_enabled = 0;
    bool front_wall_fix_enabled = 0;
    bool wall_avoid_enabled = 0;
    bool wall_theta_fix_enabled = 0;
    bool wall_cut_enabled = 0;
    float v_max = 720;
    float a_max = 3600;
    float j_max = 240000;
    float fan_duty = 0.4;
    std::array<float, field::ShapeIndexMax> v_slalom;

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
        v_slalom[i] = field::shapes[i].v_ref;
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
  enum TaskAction : char {
    TaskActionSearchRun = 'S',
    TaskActionFastRun = 'F',
    TaskActionPositionRecovery = 'P',
  };

public:
  MoveAction(const ctrl::TrajectoryTracker::Gain &gain) : tt_gain(gain) {
    for (auto &vs : rp_search.v_slalom)
      vs = v_search;
    // rp_fast.v_slalom[field::ShapeIndex::FS90] = v_search;
    for (auto &vs : rp_fast.v_slalom)
      vs = v_search;
    // vs = std::max(vs, v_search);
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "MoveAction", 8192, this, 4, NULL);
  }
  void enable(const TaskAction ta) {
    ma_log << "enable: " << (char)ta << std::endl;
    sc.disable();
    task_action = ta;
    in_action = true;
    break_requested = false;
    enabled = true;
  }
  void disable() {
    ma_log << "disable" << std::endl;
    break_requested = true;
    while (enabled)
      vTaskDelay(pdMS_TO_TICKS(1));
    sc.disable();
    while (!sa_queue.empty())
      sa_queue.pop();
    in_action = false;
    break_requested = false;
  }
  void waitForEndAction() const {
    ma_log << "wait for end action" << std::endl;
    while (in_action)
      vTaskDelay(pdMS_TO_TICKS(1));
  }
  void enqueue_action(const MazeLib::RobotBase::SearchAction action) {
    ma_log << "queue action: " << (char)action << std::endl;
    sa_queue.push(action);
    in_action = true;
    ma_log << "queue size: " << sa_queue.size() << std::endl;
  }
  void set_fast_path(const std::string &fast_path) {
    ma_log << fast_path << std::endl;
    this->fast_path = fast_path;
    in_action = true;
  }
  void emergency_release() {
    if (mt.is_emergency()) {
      bz.play(Buzzer::EMERGENCY);
      sc.disable();
      fan.drive(0);
      vTaskDelay(pdMS_TO_TICKS(400));
      mt.emergency_release();
      tof.enable();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  const auto &getSensedWalls() const { return is_wall; }
  void calibration() {
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    enc.clearOffset();
  }

  void task() {
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1));
      if (!enabled)
        continue;
      switch (task_action) {
      case TaskAction::TaskActionSearchRun:
        search_run_task();
        break;
      case TaskAction::TaskActionFastRun:
        fast_run(fast_path);
        break;
      case TaskAction::TaskActionPositionRecovery:
        position_recovery();
        break;
      default:
        break;
      }
      while (!sa_queue.empty())
        sa_queue.pop();
      enabled = false;
      in_action = false;
      break_requested = false;
    }
  }

public:
  RunParameter rp_search;
  RunParameter rp_fast;
  ctrl::TrajectoryTracker::Gain tt_gain;
  bool continue_straight_if_no_front_wall = false;

private:
  TaskAction task_action = TaskActionSearchRun;
  std::atomic_bool enabled{false};
  std::atomic_bool in_action{false};
  std::atomic_bool break_requested{false};
  lime62::concurrent_queue<MazeLib::RobotBase::SearchAction> sa_queue;
  ctrl::Pose offset;
  std::array<bool, 3> is_wall;
  bool prev_wall[2];

  bool isAlong() { return int(std::abs(offset.th) * 180 / M_PI + 1) % 90 < 2; }
  bool isDiag() {
    return int(std::abs(offset.th) * 180 / M_PI + 45 + 1) % 90 < 2;
  }

  void wall_attach(bool force = false) {
    ma_log << "wall_attach" << std::endl;
#if 1
    if (break_requested || mt.is_emergency())
      return;
    if ((force && tof.getDistance() < field::SegWidthFull * 5 / 4) ||
        tof.getDistance() < 90 ||
        (wd.distance.front[0] > 0 && wd.distance.front[1] > 0)) {
      led = 6;
      tof.disable();
      vTaskDelay(pdMS_TO_TICKS(20)); /*< ノイズ防止のためToFを無効化 */
      sc.est_p.clear();
      for (int i = 0; i < 2000; i++) {
        if (break_requested || mt.is_emergency())
          break;
        sc.sampling_sync();
        WheelParameter wp;
        for (int j = 0; j < 2; ++j)
          wp.wheel[j] = -wd.distance.front[j] * model::wall_attach_gain_Kp;
        const float end = model::wall_attach_end;
        if (math_utils::sum_of_square(wp.wheel[0], wp.wheel[1]) < end)
          break;
        wp.wheel2pole();
        const float sat_tra = 180.0f;   //< [mm/s]
        const float sat_rot = M_PI / 2; //< [rad/s]
        sc.set_target(math_utils::saturate(wp.tra, sat_tra),
                      math_utils::saturate(wp.rot, sat_rot));
      }
      sc.set_target(0, 0);
      sc.est_p.x = 0;  //< 直進方向の補正
      sc.est_p.th = 0; //< 回転方向の補正
      if (force)
        sc.est_p.y = 0; //< 強制の場合は大きくずれうる
      tof.enable();
      led = 0;
    }
#endif
  }
  void wall_avoid(const float remain, const RunParameter &rp, float &int_y) {
    if (break_requested || mt.is_emergency())
      return;
    /* 有効 かつ 一定速度より大きい かつ 姿勢が整っているときのみ */
    if (!rp.wall_avoid_enabled || sc.est_v.tra < 180.0f ||
        std::abs(sc.est_p.th) > M_PI * 0.01f) {
      led = 0;
      return;
    }
    uint8_t led_flags = 0;
    /* 90 [deg] の倍数 */
    if (isAlong()) {
      const float gain = (wd.is_wall[0] && wd.is_wall[1])
                             ? model::wall_avoid_gain
                             : model::wall_avoid_gain * 2;
      const float wall_diff_thr = 100; //< 吸い込まれ防止
      if (wd.is_wall[0] && std::abs(wd.diff.side[0]) < wall_diff_thr) {
        sc.est_p.y += wd.distance.side[0] * gain;
        int_y += wd.distance.side[0];
        led_flags |= 8;
      }
      if (wd.is_wall[1] && std::abs(wd.diff.side[1]) < wall_diff_thr) {
        sc.est_p.y -= wd.distance.side[1] * gain;
        int_y -= wd.distance.side[1];
        led_flags |= 1;
      }
      /* 機体姿勢の補正 */
      if (rp.wall_theta_fix_enabled) {
        sc.est_p.th += int_y * 1e-8f;
      }
#if 0
      /* 櫛の壁制御 */
      if (tof.getDistance() > field::SegWidthFull * 3 / 2) {
        const float comb_threashold = -54.0f;
        const float comb_shift = 0.1f;
        if (wd.distance.front[0] > comb_threashold) {
          sc.est_p.y += comb_shift;
          led_flags |= 4;
          bz.play(Buzzer::SHORT7);
        }
        if (wd.distance.front[1] > comb_threashold) {
          sc.est_p.y -= comb_shift;
          led_flags |= 2;
          bz.play(Buzzer::SHORT7);
        }
      }
#endif
    }
    /* 45 [deg] の倍数 */
    if (isDiag() && remain > field::SegWidthFull / 3) {
      const float shift = 0.06f;
      const float threashold = -50;
      if (wd.distance.front[0] > threashold) {
        sc.est_p.y += shift;
        led_flags |= 4;
      }
      if (wd.distance.front[1] > threashold) {
        sc.est_p.y -= shift;
        led_flags |= 2;
      }
    }
    led = led_flags;
  }
  void wall_cut(const RunParameter &rp) {
    if (!rp.wall_cut_enabled)
      return;
    /* 曲線なら前半しか使わない */
    if (std::abs(sc.est_p.th) > M_PI * 0.01f)
      return;
    /* 左右 */
    for (int i = 0; i < 2; i++) {
      /* 壁の変化 */
      if (prev_wall[i] && !wd.is_wall[i]) {
        /* 90 [deg] の倍数 */
        if (isAlong()) {
          const float wall_cut_offset = -15; /*< 大きいほど前へ */
          const float x_abs = offset.rotate(offset.th).x + sc.est_p.x;
          const float x_abs_cut =
              math_utils::round2(x_abs, field::SegWidthFull);
          const float fixed_x = x_abs_cut - x_abs + wall_cut_offset;
          const auto fixed_x_abs = std::abs(fixed_x);
          if (fixed_x_abs < 3.0f) {
            sc.fix_pose(ctrl::Pose(fixed_x, 0, 0));
            bz.play(Buzzer::SHORT8);
          } else if (fixed_x_abs < 10.0f) {
            sc.fix_pose(ctrl::Pose(fixed_x, 0, 0));
            bz.play(Buzzer::SHORT7);
          } else if (fixed_x_abs < 20.0f) {
            sc.fix_pose(ctrl::Pose(fixed_x / 2, 0, 0));
            bz.play(Buzzer::SHORT6);
          } else {
            bz.play(Buzzer::CANCEL);
          }
        }
        /* 45 [deg] + 90 [deg] の倍数 */
        if (isDiag()) {
          const float wall_cut_offset = -9; /*< 大きいほど前に */
                                            /* 壁切れ方向 */
          const float th_ref = (i == 0) ? (-M_PI / 4) : (M_PI / 4);
          /* 壁に沿った方向の位置 */
          const float x_abs =
              offset.rotate(offset.th + th_ref).x + sc.est_p.rotate(th_ref).x;
          const float x_abs_cut =
              math_utils::round2(x_abs, field::SegWidthFull);
          const float fixed_x = x_abs_cut - x_abs + wall_cut_offset;
          const auto fixed_x_abs = std::abs(fixed_x);
          if (fixed_x_abs < 5.0f) {
            sc.fix_pose(ctrl::Pose(fixed_x, 0, 0).rotate(-th_ref));
            bz.play(Buzzer::SHORT8);
          } else if (fixed_x_abs < 10.0f) {
            sc.fix_pose(ctrl::Pose(fixed_x, 0, 0).rotate(-th_ref));
            bz.play(Buzzer::SHORT7);
          } else if (fixed_x_abs < 20.0f) {
            sc.fix_pose(ctrl::Pose(fixed_x / 2, 0, 0).rotate(-th_ref));
            bz.play(Buzzer::SHORT6);
          } else {
            bz.play(Buzzer::CANCEL);
          }
        }
      }
      prev_wall[i] = wd.is_wall[i];
    }
  }
  void front_wall_fix(const RunParameter &rp, const float dist_to_wall_ref) {
    if (!rp.front_wall_fix_enabled)
      return;
    if (!tof.isValid() || std::abs(sc.est_p.th) > M_PI * 0.05f)
      return;
    const float wall_fix_offset = model::wall_fix_offset; /*< 大: 壁から遠く */
    const float dist_to_wall = dist_to_wall_ref - sc.est_p.x + wall_fix_offset;
    const float fixed_x_now = dist_to_wall - tof.getLog()[0] +
                              (tof.passedTimeMs() + 0) * 1e-3f * sc.ref_v.tra;
    const float fixed_x_pre = dist_to_wall - tof.getLog()[1] +
                              (tof.passedTimeMs() + 10) * 1e-3f * sc.ref_v.tra;
    /* 誤差の小さい方を選ぶ */
    const auto fixed_x = std::abs(fixed_x_now) < std::abs(fixed_x_pre)
                             ? fixed_x_now
                             : fixed_x_pre;
    const auto fixed_x_abs = std::abs(fixed_x);
    if (fixed_x_abs < 3.0f) {
      sc.fix_pose(ctrl::Pose(fixed_x, 0, 0));
      bz.play(Buzzer::SHORT8);
    } else if (fixed_x_abs < 10.0f) {
      sc.fix_pose(ctrl::Pose(fixed_x, 0, 0));
      bz.play(Buzzer::SHORT7);
    } else if (fixed_x_abs < 20.0f) {
      sc.fix_pose(ctrl::Pose(fixed_x / 2, 0, 0));
      bz.play(Buzzer::SHORT6);
    } else if (std::abs(fixed_x_pre) < 30.0f && std::abs(fixed_x_now) < 30.0f) {
      sc.fix_pose(ctrl::Pose(fixed_x / 2, 0, 0));
      bz.play(Buzzer::CANCEL);
    }
  }
  bool front_wall_fix_in_trace(const RunParameter &rp,
                               const float dist_to_wall_ref) {
    if (!rp.front_wall_fix_enabled)
      return false;
    if (!tof.isValid())
      return false;
    /* 現在の姿勢が区画に対して垂直か調べる */
    const auto th_abs = offset.th + sc.est_p.th;
    const auto th_abs_to_wall = math_utils::round2(th_abs, M_PI / 2);
    if (std::abs(th_abs - th_abs_to_wall) > 1e-3f * M_PI)
      return false;
    /* 補正開始 */
    const auto rotate_th =
        th_abs_to_wall - offset.th; /*< sc.est_p 座標系での回転量 */
    const auto pose_abs = offset + sc.est_p.rotate(offset.th);
    const auto abs_x_to_wall = pose_abs.rotate(th_abs_to_wall).x;
    const auto pos_x =
        abs_x_to_wall - math_utils::round2(abs_x_to_wall, field::SegWidthFull);
    const float wall_fix_offset = 6; /*< 調整値．大きく:前壁から遠く */
    const float dist_to_wall = dist_to_wall_ref - pos_x + wall_fix_offset;
    const float fixed_x_now = dist_to_wall - tof.getLog()[0] +
                              (tof.passedTimeMs() + 0) * 1e-3f * sc.ref_v.tra;
    const float fixed_x_pre = dist_to_wall - tof.getLog()[1] +
                              (tof.passedTimeMs() + 10) * 1e-3f * sc.ref_v.tra;
    /* 誤差の小さい方を選ぶ */
    const auto fixed_x = std::abs(fixed_x_now) < std::abs(fixed_x_pre)
                             ? fixed_x_now
                             : fixed_x_pre;
    const auto fixed_x_abs = std::abs(fixed_x);
    if (fixed_x_abs < 3.0f) {
      sc.fix_pose(ctrl::Pose(fixed_x, 0, 0).rotate(rotate_th));
      bz.play(Buzzer::SHORT8);
    } else if (fixed_x_abs < 5.0f) {
      sc.fix_pose(ctrl::Pose(fixed_x, 0, 0).rotate(rotate_th));
      bz.play(Buzzer::SHORT7);
    } else if (fixed_x_abs < 10.0f) {
      sc.fix_pose(ctrl::Pose(fixed_x / 2, 0, 0).rotate(rotate_th));
      bz.play(Buzzer::SHORT6);
    } else if (std::abs(fixed_x_pre) < 20.0f && std::abs(fixed_x_now) < 20.0f) {
      sc.fix_pose(ctrl::Pose(fixed_x / 3, 0, 0));
      bz.play(Buzzer::CANCEL);
      return false;
    }
    return true;
  }
  void straight_x(const float distance, float v_max, float v_end,
                  const RunParameter &rp, bool unknown_accel = false) {
    ma_log << "straight_x" << std::endl;
    if (break_requested || mt.is_emergency())
      return;
    v_end = unknown_accel ? v_unknown_accel : v_end;
    v_max = unknown_accel ? v_unknown_accel : v_max;
    if (distance - sc.est_p.x > 0) {
      const float v_start = sc.ref_v.tra;
      ctrl::TrajectoryTracker tt{tt_gain};
      ctrl::State ref_s;
      ctrl::straight::Trajectory trajectory;
      /* start */
      trajectory.reset(rp.j_max, rp.a_max, v_max, v_start,
                       std::max(v_end, 30.0f), distance - sc.est_p.x,
                       sc.est_p.x);
      tt.reset(v_start);
      float int_y = 0; //< 角度補正用
      for (float t = 0; true; t += sc.Ts) {
        if (break_requested || mt.is_emergency())
          break;
        /* 終了条件 */
        const float remain = distance - sc.est_p.x;
        if (remain < 0 || t > trajectory.t_end() + 0.01f)
          break; //< 静止の場合を考慮した条件
        /* 前壁制御 */
        if (isAlong() && tof.isValid()) {
          const float tof_mm = tof.getLog().average(2);
          /* 衝突被害軽減ブレーキ(AEBS) */
          if (remain > field::SegWidthFull && tof_mm < field::SegWidthFull)
            wall_stop_aebs();
          if (v_end > 1 && tof_mm < field::SegWidthFull / 2)
            wall_stop_aebs();
          /* 未知区間加速 */
          if (unknown_accel && tof_mm < 2.1f * field::SegWidthFull) {
            unknown_accel = false;
            trajectory.reset(rp.j_max, rp.a_max, v_search, sc.ref_v.tra,
                             v_search, remain, sc.est_p.x, t);
            bz.play(Buzzer::MAZE_BACKUP);
          }
        }
        /* 情報の更新 */
        sc.sampling_sync();
        /* 壁制御 */
        wall_avoid(remain, rp, int_y);
        /* 壁切れ */
        wall_cut(rp);
        /* ToDo: 補正結果に応じて時刻をシフトする処理 */
        /* 軌道追従 */
        trajectory.update(ref_s, t);
        const auto ref = tt.update(sc.est_p, sc.est_v, sc.est_a, ref_s);
        sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      }
    }
    if (v_end < 1) {
      sc.set_target(0, 0);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    /* 移動した量だけ位置を更新 */
    sc.est_p.x -= distance;
    offset += ctrl::Pose(distance, 0, 0).rotate(offset.th);
  }
  void turn(const float angle) {
    ma_log << "turn" << std::endl;
    if (break_requested || mt.is_emergency())
      return;
    const float dddth_max = 2400 * M_PI;
    const float ddth_max = 54 * M_PI;
    const float dth_max = 4 * M_PI;
    constexpr float back_gain = model::turn_back_gain;
    ctrl::AccelDesigner ad(dddth_max, ddth_max, dth_max, 0, 0, angle);
    for (float t = 0; t < ad.t_end(); t += 1e-3f) {
      if (break_requested || mt.is_emergency())
        break;
      sc.sampling_sync();
      const float delta = sc.est_p.x * std::cos(-sc.est_p.th) -
                          sc.est_p.y * std::sin(-sc.est_p.th);
      sc.set_target(-delta * back_gain, ad.v(t), 0, ad.a(t));
    }
    /* 確実に目標角度に持っていく処理 */
    float int_error = 0;
    while (1) {
      if (break_requested || mt.is_emergency())
        break;
      sc.sampling_sync();
      float delta = sc.est_p.x * std::cos(-sc.est_p.th) -
                    sc.est_p.y * std::sin(-sc.est_p.th);
      const float Kp = 10.0f;
      const float Ki = 10.0f;
      const float error = angle - sc.est_p.th;
      int_error += error * 0.001f;
      sc.set_target(-delta * back_gain, Kp * error + Ki * int_error);
      if (std::abs(Kp * error) + std::abs(Ki * int_error) < 0.1f * M_PI)
        break;
    }
    /* 移動した量だけ位置を更新 */
    const auto net = ctrl::Pose(0, 0, angle);
    sc.est_p = (sc.est_p - net).rotate(-net.th);
    offset += net.rotate(offset.th);
  }
  void trace(ctrl::slalom::Trajectory &trajectory, const RunParameter &rp) {
    ma_log << "trace" << std::endl;
    if (break_requested || mt.is_emergency())
      return;
    const float Ts = 1e-3f;
    const float velocity = sc.ref_v.tra;
    ctrl::TrajectoryTracker tt(tt_gain);
    ctrl::State s;
    /* start */
    tt.reset(velocity);
    trajectory.reset(velocity);
    bool front_fix_ready = rp.front_wall_fix_enabled; /*< ターン中の前壁修正 */
    s.q.x = sc.est_p.x; /*< 既に移動した分を反映 */
    if (std::abs(sc.est_p.x) > 1)
      bz.play(Buzzer::CONFIRM);
    for (float t = 0; t < trajectory.getTimeCurve(); t += Ts) {
      if (break_requested || mt.is_emergency())
        break;
      /* データの更新 */
      sc.sampling_sync();
      /* 補正 */
      float int_y = 0; //< スラローム中は角度補正をしない
      wall_avoid(0, rp, int_y);
      wall_cut(rp);
      /* ターン中の前壁補正 */
      const auto &shape = trajectory.getShape();
      if (front_fix_ready && t > trajectory.getTimeCurve() / 4)
        if (&shape != &field::shapes[field::ShapeIndex::FS90])
          front_fix_ready = !front_wall_fix_in_trace(rp, field::SegWidthFull);
      /* ToDo: 補正結果によって t をシフト*/
      /* 軌道を更新 */
      trajectory.update(s, t, Ts);
      const auto ref = tt.update(sc.est_p, sc.est_v, sc.est_a, s);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      /* ToDo: 打ち切り条件を追加！！！ */
    }
    sc.set_target(velocity, 0);
    /* 移動した量だけ位置を更新 */
    const auto &net = trajectory.getShape().curve;
    sc.est_p = (sc.est_p - net).rotate(-net.th);
    offset += net.rotate(offset.th);
  }
  void SlalomProcess(const field::ShapeIndex si, const bool mirror_x,
                     const bool reverse, float &straight,
                     const RunParameter &rp) {
    if (break_requested || mt.is_emergency())
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
    /* 直線前壁補正 */
    if (isAlong()) {
      if (rp.diag_enabled && reverse == false) {
        front_wall_fix(rp, field::SegWidthFull * 3 / 2 - straight_prev);
        front_wall_fix(rp, field::SegWidthFull * 5 / 2 - straight_prev);
      }
      if (si == field::ShapeIndex::FS90) {
        front_wall_fix(rp, field::SegWidthFull - straight_prev);
        front_wall_fix(rp, 2 * field::SegWidthFull - straight_prev);
      }
      /* 前壁制御で発生した直線を走行 */
      straight_x(0, velocity, velocity, rp);
    }
    /* スラローム */
    trace(st, rp);
    straight += reverse ? straight_prev : straight_post;
  }
  void u_turn() {
    if (break_requested || mt.is_emergency())
      return;
    if (wd.distance.side[0] < wd.distance.side[1]) {
      wall_attach();
      turn(-M_PI / 2);
      wall_attach();
      turn(-M_PI / 2);
    } else {
      wall_attach();
      turn(M_PI / 2);
      wall_attach();
      turn(M_PI / 2);
    }
  }
  void wall_stop_aebs() {
    ma_log << "wall_stop_aebs" << std::endl;
    if (break_requested || mt.is_emergency())
      return;
    bz.play(Buzzer::AEBS);
    // ToDo: compiler bug avoidance!
    for (float v = sc.ref_v.tra; v > 0; v -= 12) {
      sc.sampling_sync();
      sc.set_target(v, 0);
    }
    sc.set_target(0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    sc.disable();
    mt.emergency_stop();
    break_requested = true;
  }
  void start_step(const RunParameter &rp) {
    ma_log << "start step" << std::endl;
    if (break_requested || mt.is_emergency())
      return;
    sc.disable();
    mt.drive(-0.2f, -0.2f); /*< 背中を確実に壁につける */
    vTaskDelay(pdMS_TO_TICKS(500));
    mt.free();
    sc.sampling_sync();
    sc.enable();
    sc.est_p.clear();
    sc.est_p.x = model::TailLength + field::WallThickness / 2;
    offset = ctrl::Pose(field::SegWidthFull / 2, 0, M_PI / 2);
    straight_x(field::SegWidthFull, v_search, v_search, rp);
  }
  void start_init() {
    ma_log << "start init" << std::endl;
    if (break_requested || mt.is_emergency())
      return;
    wall_attach();
    turn(M_PI / 2);
    wall_attach();
    turn(M_PI / 2);
    put_back();
    mt.free();
    sc.disable();
    break_requested = true;
  }
  void put_back() {
    if (break_requested || mt.is_emergency())
      return;
    const int max_v = 150;
    const float th_gain = 50.0f;
    for (int i = 0; i < max_v; i++) {
      sc.set_target(-i, -sc.est_p.th * th_gain);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 100; i++) {
      sc.set_target(-max_v, -sc.est_p.th * th_gain);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    mt.drive(-0.1f, -0.1f);
    vTaskDelay(pdMS_TO_TICKS(200));
    mt.drive(-0.2f, -0.2f);
    vTaskDelay(pdMS_TO_TICKS(200));
    mt.drive(0, 0);
  }

private:
  void search_run_task() {
    ma_log << "search_run_task" << std::endl;
    const auto &rp = rp_search;
    /* スタート */
    sc.enable();
    /* 区画の中心に配置 */
    offset = ctrl::Pose(field::SegWidthFull / 2,
                        field::SegWidthFull / 2 + model::CenterShift, M_PI / 2);
    while (1) {
      if (break_requested || mt.is_emergency())
        break;
      /* 壁を確認 */
      is_wall = wd.is_wall;
      ma_log << "get wall" << std::endl;
      /* 探索器に終了を通知 */
      if (sa_queue.empty())
        in_action = false;
      /* Actionがキューされるまで直進で待つ */
      search_run_queue_wait_decel(rp);
      /* 既知区間走行 */
      if (sa_queue.size() >= 2)
        search_run_known(rp);
      /* 探索走行 */
      if (!sa_queue.empty()) {
        const auto action = sa_queue.front();
        sa_queue.pop();
        search_run_switch(action, rp);
      }
    }
    sc.disable();
  }
  void search_run_queue_wait_decel(const RunParameter &rp) {
    ma_log << "wait decel" << std::endl;
    /* Actionがキューされるまで減速しながら待つ */
    ctrl::TrajectoryTracker tt(tt_gain);
    ctrl::State ref_s;
    const auto v_start = sc.ref_v.tra;
    ctrl::AccelCurve ac(rp.j_max, rp.a_max, v_start, 0); //< なめらかに減速
    /* start */
    tt.reset(v_start);
    float int_y = 0; //< 角度補正
    for (float t = 0; sa_queue.empty(); t += 1e-3f) {
      if (break_requested || mt.is_emergency())
        break;
      sc.sampling_sync();
      int_y = 0; //< 角度補正はしない
      wall_avoid(0, rp, int_y);
      const auto ref = tt.update(sc.est_p, sc.est_v, sc.est_a,
                                 ctrl::Pose(ac.x(t)), ctrl::Pose(ac.v(t)),
                                 ctrl::Pose(ac.a(t)), ctrl::Pose(ac.j(t)));
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
    }
    /* 注意: 現在位置はやや前に進んだ状態 */
  }
  void search_run_known(const RunParameter &rp) {
    ma_log << "search run known" << std::endl;
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
    ma_log << "known path: " << path << std::endl;
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
      /* 最後の直線を消化 */
      if (straight > 0.1f) {
        straight_x(straight, rp.v_max, v_search, rp);
        straight = 0;
      }
    }
  }
  void search_run_switch(const MazeLib::RobotBase::SearchAction action,
                         const RunParameter &rp) {
    ma_log << "search run switch: " << (char)action << std::endl;
    if (break_requested || mt.is_emergency())
      return;
    const bool no_front_front_wall =
        tof.getDistance() > field::SegWidthFull * 2 + field::SegWidthFull / 2;
    const bool unknown_accel = rp.unknown_accel_enabled &&
                               continue_straight_if_no_front_wall &&
                               no_front_front_wall;
    switch (action) {
    case MazeLib::RobotBase::SearchAction::START_STEP:
      start_step(rp);
      break;
    case MazeLib::RobotBase::SearchAction::START_INIT:
      start_init();
      break;
    case MazeLib::RobotBase::SearchAction::ST_FULL:
      if (tof.getDistance() < field::SegWidthFull)
        return wall_stop_aebs();
      front_wall_fix(rp, 2 * field::SegWidthFull);
      straight_x(field::SegWidthFull, v_search, v_search, rp, unknown_accel);
      break;
    case MazeLib::RobotBase::SearchAction::ST_HALF:
      straight_x(field::SegWidthFull / 2 - model::CenterShift, v_search,
                 v_search, rp);
      break;
    case MazeLib::RobotBase::SearchAction::TURN_L:
      front_wall_fix(rp, field::SegWidthFull);
      front_wall_fix(rp, 2 * field::SegWidthFull);
      if (sc.est_p.x < 5.0f && sc.ref_v.tra < v_search * 1.2f) {
        ctrl::slalom::Trajectory st(field::shapes[field::ShapeIndex::S90], 0);
        straight_x(st.getShape().straight_prev, v_search, v_search, rp);
        if (wd.is_wall[0])
          return wall_stop_aebs();
        trace(st, rp);
        straight_x(st.getShape().straight_post, v_search, v_search, rp);
      } else {
        straight_x(field::SegWidthFull / 2 + model::CenterShift, v_search, 0,
                   rp);
        wall_attach();
        turn(M_PI / 2);
        straight_x(field::SegWidthFull / 2 - model::CenterShift, v_search,
                   v_search, rp);
      }
      break;
    case MazeLib::RobotBase::SearchAction::TURN_R:
      front_wall_fix(rp, field::SegWidthFull);
      front_wall_fix(rp, 2 * field::SegWidthFull);
      if (sc.est_p.x < 5.0f && sc.ref_v.tra < v_search * 1.2f) {
        ctrl::slalom::Trajectory st(field::shapes[field::ShapeIndex::S90], 1);
        straight_x(st.getShape().straight_prev, v_search, v_search, rp);
        if (wd.is_wall[1])
          return wall_stop_aebs();
        trace(st, rp);
        straight_x(st.getShape().straight_post, v_search, v_search, rp);
      } else {
        straight_x(field::SegWidthFull / 2 + model::CenterShift, v_search, 0,
                   rp);
        wall_attach();
        turn(-M_PI / 2);
        straight_x(field::SegWidthFull / 2 - model::CenterShift, v_search,
                   v_search, rp);
      }
      break;
    case MazeLib::RobotBase::SearchAction::ROTATE_180:
      u_turn();
      break;
    case MazeLib::RobotBase::SearchAction::ST_HALF_STOP:
      front_wall_fix(rp, field::SegWidthFull);
      front_wall_fix(rp, 2 * field::SegWidthFull);
      straight_x(field::SegWidthFull / 2 + model::CenterShift, v_search, 0, rp);
      break;
    }
  }

private:
  std::string fast_path;

  bool fast_run(const std::string &search_actions) {
    ma_log << "fast_run" << std::endl;
    /* 走行パラメータを取得 */
    const auto &rp = rp_fast;
    /* 最短走行用にパターンを置換 */
    const auto path = MazeLib::RobotBase::pathConvertSearchToFast(
        search_actions, rp.diag_enabled);
    /* キャリブレーション */
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    /* 壁に背中を確実につける */
    mt.drive(-0.25f, -0.25f);
    vTaskDelay(pdMS_TO_TICKS(200));
    mt.free();
    /* ファンを始動 */
    fan.drive(rp.fan_duty);
    vTaskDelay(pdMS_TO_TICKS(400)); //< ファンの回転数が一定なるのを待つ
    /* 走行開始 */
    sc.enable();
    /* 初期位置を設定 */
    offset = ctrl::Pose(field::SegWidthFull / 2,
                        model::TailLength + field::WallThickness / 2, M_PI / 2);
    /* 最初の直線を追加 */
    float straight =
        field::SegWidthFull / 2 - model::TailLength - field::WallThickness / 2;
    /* 走行 */
    for (int path_index = 0; path_index < path.length(); path_index++) {
      if (break_requested || mt.is_emergency())
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
    sc.set_target(0, 0);
    fan.drive(0);
    if (!mt.is_emergency())
      vTaskDelay(pdMS_TO_TICKS(200));
    sc.disable();
    /* クラッシュ時の処理 */
    if (!mt.is_emergency())
      bz.play(Buzzer::COMPLETE);
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
    static constexpr float dddth_max = 4800 * M_PI;
    static constexpr float ddth_max = 48 * M_PI;
    static constexpr float dth_max = 2 * M_PI;
    const float angle = 2 * M_PI;
    ctrl::AccelDesigner ad(dddth_max, ddth_max, dth_max, 0, 0, angle);
    /* 走査データ格納用変数 */
    constexpr int table_size = 96;
    std::bitset<table_size> is_valid;
    std::array<uint16_t, table_size> table;
    table.fill(255);
    /* start */
    sc.enable();
    int index = 0;
    for (float t = 0; t < ad.t_end(); t += 1e-3f) {
      if (break_requested || mt.is_emergency())
        break;
      sc.sampling_sync();
      const float delta = sc.est_p.x * std::cos(-sc.est_p.th) -
                          sc.est_p.y * std::sin(-sc.est_p.th);
      constexpr float back_gain = model::turn_back_gain;
      sc.set_target(-delta * back_gain, ad.v(t), 0, ad.a(t));
      if (ad.x(t) > 2 * M_PI * index / table_size) {
        index++;
        table[index % table_size] = tof.getDistance();
        is_valid[index % table_size] = tof.isValid();
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
    sc.est_p.clear();
    turn(2 * M_PI * min_i / table_size);
    /* 壁が遠い場合は直進する */
    if (tof.isValid() && tof.getDistance() > field::SegWidthFull / 2)
      straight_x(tof.getDistance() - field::SegWidthFull / 2, 240, 0,
                 rp_search);
    /* 前壁補正 */
    wall_attach(true);
    turn(wd.distance.side[0] > wd.distance.side[1] ? M_PI / 2 : -M_PI / 2);
    /* 壁のない方向を向く */
    while (!mt.is_emergency()) {
      if (tof.getDistance() > field::SegWidthFull)
        break;
      wall_attach(true);
      turn(-M_PI / 2);
    }
    sc.disable();
    // if (mt.is_emergency()) {
    //   bz.play(Buzzer::EMERGENCY);
    //   mt.emergency_release();
    //   return false;
    // }
    return true;
  }
};
