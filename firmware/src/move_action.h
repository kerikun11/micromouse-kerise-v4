#pragma once

#include "config/model.h"
#include "global.h"

#include "accel_designer.h"
#include "slalom.h"
#include "slalom_shapes.h"
#include "straight.h"
#include "trajectory_tracker.h"

#include <TaskBase.h>

#include <cmath>
#include <queue>

#define SEARCH_WALL_ATTACH_ENABLED 1

#define SEARCH_RUN_TASK_PRIORITY 3
#define SEARCH_RUN_STACK_SIZE 8192

#include <RobotBase.h>
using namespace MazeLib;

class MoveAction : TaskBase {
public:
  static constexpr float unknown_accel_velocity = 540;
  static constexpr float v_search = 300;

public:
  struct RunParameter {
  public:
    bool diag_enabled = 1;
    bool unknown_accel_enabled = 0;
    bool front_wall_fix_enabled = 1;
    bool front_wall_fix_trace_enabled = 1;
    bool wall_avoid_enabled = 1;
    bool wall_theta_fix_enabled = 1;
    bool wall_cut_enabled = 0;
    float curve_gain = 1.0;
    float max_speed = 720;
    float accel = 3600;
    float jerk = 240000;
    float fan_duty = 0.2f;

  public:
    // [1*1.05**i for i in range(0, 4)]: [1.0, 1.05, 1.1025, 1.1576]
    static constexpr float cg_gain = 1.05f;
    // [int(720*1.2**i) for i in range(0, 4)]: [720, 864, 1036, 1244]
    static constexpr float ms_gain = 1.1f;
    // [int(3600*1.05**i) for i in range(0, 4)]: [3600, 3780, 3969, 4167]
    static constexpr float ac_gain = 1.05f;

  public:
    void up(const int cnt = 1) {
      for (int i = 0; i < cnt; ++i) {
        curve_gain *= cg_gain;
        max_speed *= ms_gain;
        accel *= ac_gain;
      }
    }
    void down(const int cnt = 1) {
      for (int i = 0; i < cnt; ++i) {
        curve_gain /= cg_gain;
        max_speed /= ms_gain;
        accel /= ac_gain;
      }
    }
  };
#ifndef M_PI
  static constexpr float M_PI = 3.14159265358979323846f;
#endif

public:
  MoveAction(const ctrl::TrajectoryTracker::Gain &gain) : tt_gain(gain) {
    // rp_search.diag_enabled = 0;
  }
  ~MoveAction() {}
  void enable() {
    deleteTask();
    isRunningFlag = true;
    createTask("MoveAction", SEARCH_RUN_TASK_PRIORITY, SEARCH_RUN_STACK_SIZE);
  }
  void disable() {
    deleteTask();
    sc.disable();
    while (q.size())
      q.pop();
    path = "";
    isRunningFlag = false;
  }
  bool isRunning() { return isRunningFlag; }
  void set_action(RobotBase::Action action) {
    q.push(action);
    isRunningFlag = true;
  }
  void set_path(std::string path) { this->path = path; }
  bool positionRecovery() {
    /* 1周回って壁を探す */
    sc.enable();
    static constexpr float m_dddth = 4800 * M_PI;
    static constexpr float m_ddth = 48 * M_PI;
    static constexpr float m_dth = 2 * M_PI;
    const float angle = 2 * M_PI;
    constexpr int table_size = 96;
    std::array<uint16_t, table_size> table;
    std::bitset<table_size> is_valid;
    table.fill(255);
    ctrl::AccelDesigner ad(m_dddth, m_ddth, 0, m_dth, 0, angle);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int index = 0;
    for (float t = 0; t < ad.t_end(); t += 0.001f) {
      float delta = sc.position.x * std::cos(-sc.position.th) -
                    sc.position.y * std::sin(-sc.position.th);
      constexpr float back_gain = 10.0f;
      sc.set_target(-delta * back_gain, ad.v(t), 0, ad.a(t));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      if (ad.x(t) > 2 * PI * index / table_size) {
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
    sc.enable(); //< reset
    turn(2 * M_PI * min_i / table_size);
    /* 壁が遠い場合は直進する */
    if (tof.isValid() && tof.getDistance() > field::SegWidthFull / 2) {
      straight_x(tof.getDistance() - field::SegWidthFull / 2, 240, 0,
                 rp_search);
    }
    /* 前壁補正 */
    wall_attach(true);
    turn(wd.distance.side[0] > wd.distance.side[1] ? M_PI / 2 : -M_PI / 2);
    delay(20); //< ToFが有効化するのを待つ
    /* 壁のない方向を向く */
    while (1) {
      if (tof.getDistance() > field::SegWidthFull)
        break;
      wall_attach(true);
      turn(-M_PI / 2);
    }
    sc.disable();
    return true;
  }

public:
  RunParameter rp_search;
  RunParameter rp_fast;
  ctrl::TrajectoryTracker::Gain tt_gain;

  /* for communication with MazeRobot */
public:
  bool continue_straight_if_no_front_wall = false;
  bool wall_stop_flag = false;
  std::array<bool, 3> is_wall;

private:
  std::queue<RobotBase::Action> q;
  bool isRunningFlag = false;
  bool isNeutralTurnMode = false;
  ctrl::Position offset;
  std::string path;
  bool prev_wall[2];

  static auto round2(auto value, auto div) {
    return floor((value + div / 2) / div) * div;
  }
  static auto saturate(auto src, auto sat) {
    return std::max(std::min(src, sat), -sat);
  }
  bool isAlong() {
    return (int)(std::abs(offset.th) * 180.0f / PI + 1) % 90 < 2;
  }
  bool isDiag() {
    return (int)(std::abs(offset.th) * 180.0f / PI + 45 + 1) % 90 < 2;
  }

  void wall_attach(bool force = false) {
#if SEARCH_WALL_ATTACH_ENABLED
    if ((force && tof.getDistance() < field::SegWidthFull * 5 / 4) ||
        tof.getDistance() < 90 ||
        (wd.distance.front[0] > 0 && wd.distance.front[1] > 0)) {
      led = 6;
      tof.disable();
      vTaskDelay(pdMS_TO_TICKS(10)); /*< ノイズ防止のためToFを無効化 */
      TickType_t xLastWakeTime = xTaskGetTickCount();
      WheelParameter wi;
      for (int i = 0; i < 2000; i++) {
        const float Kp = model::wall_attach_gain_Kp;
        const float Ki = model::wall_attach_gain_Ki;
        const float sat_integral = 30.0f;
        const float end = model::wall_attach_end;
        WheelParameter wp;
        for (int j = 0; j < 2; ++j) {
          wp.wheel[j] = -wd.distance.front[j];
          wi.wheel[j] += wp.wheel[j] * 0.001f * Ki;
          wi.wheel[j] = saturate(wi.wheel[j], sat_integral);
          wp.wheel[j] = wp.wheel[j] * Kp + wi.wheel[j];
        }
        if (std::pow(wp.wheel[0], 2) + std::pow(wp.wheel[1], 2) +
                std::pow(wi.wheel[0], 2) + std::pow(wi.wheel[1], 2) <
            end)
          break;
        wp.wheel2pole();
        const float sat_tra = 120.0f;   //< [mm/s]
        const float sat_rot = M_PI / 4; //< [rad/s]
        sc.set_target(saturate(wp.tra, sat_tra), saturate(wp.rot, sat_rot));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      }
      sc.set_target(0, 0);
      sc.position.x = 0;  //< 直進方向の補正
      sc.position.th = 0; //< 回転方向の補正
      if (force)
        sc.position.y = 0; //< 強制の場合は大きくずれうる
      tof.enable();
      led = 0;
    }
#endif
  }
  void wall_avoid(const float remain, const RunParameter &rp, float &int_y) {
    /* 有効 かつ 一定速度より大きい かつ 姿勢が整っているときのみ */
    if (!rp.wall_avoid_enabled || sc.est_v.tra < 180.0f ||
        std::abs(sc.position.th) > M_PI * 0.1f)
      return;
    uint8_t led_flags = 0;
    /* 90 [deg] の倍数 */
    if (isAlong()) {
      const float gain = (wd.is_wall[0] && wd.is_wall[1])
                             ? model::wall_avoid_gain
                             : model::wall_avoid_gain * 2;
      const float wall_diff_thr = 100; //< 吸い込まれ防止
      if (wd.is_wall[0] && std::abs(wd.diff.side[0]) < wall_diff_thr) {
        sc.position.y += wd.distance.side[0] * gain;
        int_y += wd.distance.side[0];
        led_flags |= 8;
      }
      if (wd.is_wall[1] && std::abs(wd.diff.side[1]) < wall_diff_thr) {
        sc.position.y -= wd.distance.side[1] * gain;
        int_y -= wd.distance.side[1];
        led_flags |= 1;
      }
      /* 機体姿勢の補正 */
      if (rp.wall_theta_fix_enabled) {
        sc.position.th += int_y * 1e-8f;
      }
      /* 櫛の壁制御 */
      if (tof.getDistance() > field::SegWidthFull * 3 / 2) {
        const float comb_threashold = -50.0f;
        const float comb_shift = 0.1f;
        if (wd.distance.front[0] > comb_threashold) {
          sc.position.y += comb_shift;
          led_flags |= 4;
          bz.play(Buzzer::SHORT7);
        }
        if (wd.distance.front[1] > comb_threashold) {
          sc.position.y -= comb_shift;
          led_flags |= 2;
          bz.play(Buzzer::SHORT7);
        }
      }
    }
    /* 45 [deg] の倍数 */
    if (isDiag() && remain > field::SegWidthFull / 3) {
      const float shift = 0.06f;
      const float threashold = -50;
      if (wd.distance.front[0] > threashold) {
        sc.position.y += shift;
        led_flags |= 4;
      }
      if (wd.distance.front[1] > threashold) {
        sc.position.y -= shift;
        led_flags |= 2;
      }
    }
    led = led_flags;
  }
  void wall_cut(const RunParameter &rp) {
    if (!rp.wall_cut_enabled)
      return;
    /* 曲線なら前半しか使わない */
    if (std::abs(sc.position.th) > M_PI * 0.01f)
      return;
    /* 左右 */
    for (int i = 0; i < 2; i++) {
      /* 壁の変化 */
      if (prev_wall[i] && !wd.is_wall[i]) {
        /* 90 [deg] の倍数 */
        if (isAlong()) {
          const float wall_cut_offset = -15; /*< 大きいほど前へ */
          const float x_abs = offset.rotate(offset.th).x + sc.position.x;
          const float x_abs_cut = round2(x_abs, field::SegWidthFull);
          const float fixed_x = x_abs_cut - x_abs + wall_cut_offset;
          const auto fixed_x_abs = std::abs(fixed_x);
          if (fixed_x_abs < 3.0f) {
            sc.fix_position(ctrl::Position(fixed_x, 0, 0));
            bz.play(Buzzer::SHORT8);
          } else if (fixed_x_abs < 10.0f) {
            sc.fix_position(ctrl::Position(fixed_x, 0, 0));
            bz.play(Buzzer::SHORT7);
          } else if (fixed_x_abs < 20.0f) {
            sc.fix_position(ctrl::Position(fixed_x / 2, 0, 0));
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
          const float x_abs = offset.rotate(offset.th + th_ref).x +
                              sc.position.rotate(th_ref).x;
          const float x_abs_cut = round2(x_abs, field::SegWidthFull);
          const float fixed_x = x_abs_cut - x_abs + wall_cut_offset;
          const auto fixed_x_abs = std::abs(fixed_x);
          if (fixed_x_abs < 5.0f) {
            sc.fix_position(ctrl::Position(fixed_x, 0, 0).rotate(-th_ref));
            bz.play(Buzzer::SHORT8);
          } else if (fixed_x_abs < 10.0f) {
            sc.fix_position(ctrl::Position(fixed_x, 0, 0).rotate(-th_ref));
            bz.play(Buzzer::SHORT7);
          } else if (fixed_x_abs < 20.0f) {
            sc.fix_position(ctrl::Position(fixed_x / 2, 0, 0).rotate(-th_ref));
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
    if (!tof.isValid() || std::abs(sc.position.th) > M_PI * 0.05f)
      return;
    const float wall_fix_offset = 8; /*< 調整値．大きく:前壁から遠く */
    const float dist_to_wall =
        dist_to_wall_ref - sc.position.x + wall_fix_offset;
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
      sc.fix_position(ctrl::Position(fixed_x, 0, 0));
      bz.play(Buzzer::SHORT8);
    } else if (fixed_x_abs < 10.0f) {
      sc.fix_position(ctrl::Position(fixed_x, 0, 0));
      bz.play(Buzzer::SHORT7);
    } else if (fixed_x_abs < 20.0f) {
      sc.fix_position(ctrl::Position(fixed_x / 2, 0, 0));
      bz.play(Buzzer::SHORT6);
    } else if (std::abs(fixed_x_pre) < 30.0f && std::abs(fixed_x_now) < 30.0f) {
      sc.fix_position(ctrl::Position(fixed_x / 2, 0, 0));
      bz.play(Buzzer::CANCEL);
    }
  }
  bool front_wall_fix_trace(const RunParameter &rp,
                            const float dist_to_wall_ref) {
    if (!rp.front_wall_fix_enabled)
      return false;
    if (!tof.isValid())
      return false;
    /* 現在の姿勢が区画に対して垂直か調べる */
    const auto th_abs = offset.th + sc.position.th;
    const auto th_abs_to_wall = round2(th_abs, M_PI / 2);
    if (std::abs(th_abs - th_abs_to_wall) > 0.001f * M_PI)
      return false;
    /* 補正開始 */
    const auto rotate_th =
        th_abs_to_wall - offset.th; /*< sc.position 座標系での回転量 */
    const auto position_abs = offset + sc.position.rotate(offset.th);
    const auto abs_x_to_wall = position_abs.rotate(th_abs_to_wall).x;
    const auto pos_x =
        abs_x_to_wall - round2(abs_x_to_wall, field::SegWidthFull);
    const float wall_fix_offset = 4; /*< 調整値．大きく:前壁から遠く */
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
      sc.fix_position(ctrl::Position(fixed_x, 0, 0).rotate(rotate_th));
      bz.play(Buzzer::SHORT8);
    } else if (fixed_x_abs < 5.0f) {
      sc.fix_position(ctrl::Position(fixed_x, 0, 0).rotate(rotate_th));
      bz.play(Buzzer::SHORT7);
    } else if (fixed_x_abs < 10.0f) {
      sc.fix_position(ctrl::Position(fixed_x / 2, 0, 0).rotate(rotate_th));
      bz.play(Buzzer::SHORT6);
    } else if (std::abs(fixed_x_pre) < 20.0f && std::abs(fixed_x_now) < 20.0f) {
      sc.fix_position(ctrl::Position(fixed_x / 3, 0, 0));
      bz.play(Buzzer::CANCEL);
      return false;
    }
    return true;
  }
  void turn(const float angle) {
    static constexpr float m_dddth = 4800 * M_PI;
    static constexpr float m_ddth = 48 * M_PI;
    static constexpr float m_dth = 4 * M_PI;
    ctrl::AccelDesigner ad(m_dddth, m_ddth, 0, m_dth, 0, angle);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const float back_gain = 10.0f;
    for (float t = 0; t < ad.t_end(); t += 0.001f) {
      float delta = sc.position.x * std::cos(-sc.position.th) -
                    sc.position.y * std::sin(-sc.position.th);
      sc.set_target(-delta * back_gain, ad.v(t), 0, ad.a(t));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
    /* 確実に目標角度に持っていく処理 */
    float int_error = 0;
    while (1) {
      float delta = sc.position.x * std::cos(-sc.position.th) -
                    sc.position.y * std::sin(-sc.position.th);
      const float Kp = 20.0f;
      const float Ki = 10.0f;
      const float error = angle - sc.position.th;
      int_error += error * 0.001f;
      sc.set_target(-delta * back_gain, Kp * error + Ki * int_error);
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      if (std::abs(Kp * error) + std::abs(Ki * int_error) < 0.05f * PI)
        break;
    }
    sc.set_target(0, 0);
    /* 移動した量だけ位置を更新 */
    const auto net = ctrl::Position(0, 0, angle);
    sc.position = (sc.position - net).rotate(-net.th);
    offset += net.rotate(offset.th);
  }
  void straight_x(const float distance, const float v_max, const float v_end,
                  const RunParameter &rp) {
    if (distance - sc.position.x > 0) {
      const float v_start = sc.ref_v.tra;
      ctrl::TrajectoryTracker tt{tt_gain};
      ctrl::State ref_s;
      ctrl::straight::Trajectory trajectory;
      /* start */
      trajectory.reset(rp.jerk, rp.accel, v_start, v_max, v_end,
                       distance - sc.position.x, sc.position.x);
      tt.reset(v_start);
      float int_y = 0;
      TickType_t xLastWakeTime = xTaskGetTickCount();
      for (float t = 0; true; t += 0.001f) {
        /* 終了条件 */
        const float remain = distance - sc.position.x;
        if (remain < 0 || t > trajectory.t_end() + 0.1f)
          break;
        /* 衝突被害軽減ブレーキ(AEBS) */
        if (isAlong() && remain > field::SegWidthFull && tof.isValid() &&
            tof.getDistance() < field::SegWidthFull)
          wall_stop();
        /* 軌道追従 */
        trajectory.update(ref_s, t);
        const auto ref = tt.update(sc.position, sc.est_v, sc.est_a, ref_s);
        sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
        /* 壁制御 */
        wall_avoid(remain, rp, int_y);
        wall_cut(rp);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      }
    }
    /* 移動した量だけ位置を更新 */
    sc.position.x -= distance;
    offset += ctrl::Position(distance, 0, 0).rotate(offset.th);
  }
  void trace(ctrl::slalom::Trajectory &trajectory, const RunParameter &rp) {
    const float Ts = 0.001f;
    const float velocity = sc.est_v.tra;
    ctrl::TrajectoryTracker tt(tt_gain);
    ctrl::State s;
    /* start */
    tt.reset(velocity);
    trajectory.reset(velocity);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool front_fix_ready =
        rp.front_wall_fix_trace_enabled; /*< ターン中の前壁修正 */
    s.q.x = sc.position.x;               /*< 既に移動した分を反映 */
    if (std::abs(sc.position.x) > 1.0f)
      bz.play(Buzzer::CONFIRM);
    for (float t = 0; t < trajectory.getTimeCurve(); t += Ts) {
      /* 打ち切り条件を追加！！！ */
      trajectory.update(s, t, Ts);
      const auto ref = tt.update(sc.position, sc.est_v, sc.est_a, s);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      float int_y = 0;
      wall_avoid(0, rp, int_y);
      wall_cut(rp);
      /* ターン中の前壁補正 */
      const auto &shape = trajectory.getShape();
      if (front_fix_ready && t > trajectory.getTimeCurve() / 4)
        if (!(&shape == &SS_FS90L) && !(&shape == &SS_FS90R))
          front_fix_ready = !front_wall_fix_trace(rp, field::SegWidthFull);
      /* 同期 */
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
    sc.set_target(velocity, 0);
    /* 移動した量だけ位置を更新 */
    const auto &net = trajectory.getShape().curve;
    sc.position = (sc.position - net).rotate(-net.th);
    offset += net.rotate(offset.th);
  }
  void SlalomProcess(const ctrl::slalom::Shape &shape, float &straight,
                     const bool reverse, const RunParameter &rp) {
    ctrl::slalom::Trajectory st(shape);
    const auto straight_prev = shape.straight_prev;
    const auto straight_post = shape.straight_post;
    const float velocity =
        path.empty() ? v_search : shape.v_ref * rp.curve_gain;
    straight += !reverse ? straight_prev : straight_post;
    /* ターン前の直線を消化 */
    if (straight > 0.1f) {
      straight_x(straight, rp.max_speed, velocity, rp);
      straight = 0;
    }
    /* 直線前壁補正 */
    if (isAlong()) {
      if (rp.diag_enabled && reverse == false) {
        front_wall_fix(rp, field::SegWidthFull + field::SegWidthFull / 2 -
                               straight_prev);
        front_wall_fix(rp, 2 * field::SegWidthFull + field::SegWidthFull / 2 -
                               straight_prev);
        /* 前壁制御で発生した直線を走行 */
        straight_x(0, velocity, velocity, rp);
      }
      if (&shape == &SS_FS90L || &shape == &SS_FS90R) {
        front_wall_fix(rp, field::SegWidthFull - straight_prev);
        front_wall_fix(rp, 2 * field::SegWidthFull - straight_prev);
      }
    }
    /* スラローム */
    trace(st, rp);
    straight += reverse ? straight_prev : straight_post;
  }
  void put_back() {
    const int max_v = 150;
    const float th_gain = 100.0f;
    for (int i = 0; i < max_v; i++) {
      sc.set_target(-i, -sc.position.th * th_gain);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 100; i++) {
      sc.set_target(-max_v, -sc.position.th * th_gain);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    sc.disable();
    mt.drive(-0.1f, -0.1f);
    vTaskDelay(pdMS_TO_TICKS(200));
    mt.drive(-0.2f, -0.2f);
    vTaskDelay(pdMS_TO_TICKS(200));
    sc.enable(true);
  }
  void uturn() {
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
  void wall_stop() {
    bz.play(Buzzer::AEBS);
    float v = sc.est_v.tra;
    while (v > 0) {
      sc.set_target(v, 0);
      v -= 12;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    sc.disable();
    wall_stop_flag = true;
    mt.emergencyStop();
    vTaskDelay(portMAX_DELAY);
  }
  void start_init() {
    wall_attach();
    turn(M_PI / 2);
    wall_attach();
    turn(M_PI / 2);
    put_back();
    mt.free();
    isRunningFlag = false;
    vTaskDelay(portMAX_DELAY);
  }
  void queue_wait_decel(const RunParameter &rp) {
    /* Actionがキューされるまで直進で待つ */
    ctrl::TrajectoryTracker tt(tt_gain);
    ctrl::State ref_s;
    const auto v_start = sc.ref_v.tra;
    ctrl::AccelCurve ac(rp.jerk, rp.accel, v_start, 0);
    /* start */
    tt.reset(v_start);
    float int_y = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (float t = 0; q.empty(); t += 0.001f) {
      /* 軌道追従 */
      const auto ref =
          tt.update(sc.position, sc.est_v, sc.est_a, ctrl::Position(ac.x(t)),
                    ctrl::Position(ac.v(t)), ctrl::Position(ac.a(t)),
                    ctrl::Position(ac.j(t)));
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      /* 壁制御 */
      wall_avoid(0, rp, int_y);
      wall_cut(rp);
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
  }
  void task() override {
    if (q.empty())
      fast_run_task();
    else
      search_run_task();
    vTaskDelay(portMAX_DELAY);
  }
  void search_run_known(const RunParameter &rp) {
    /* path の作成 */
    std::string path;
    while (1) {
      if (q.empty())
        break;
      const auto action = q.front();
      if (action == MazeLib::RobotBase::Action::ST_HALF ||
          action == MazeLib::RobotBase::Action::ST_FULL ||
          action == MazeLib::RobotBase::Action::TURN_L ||
          action == MazeLib::RobotBase::Action::TURN_R) {
        path += action;
        q.pop();
      } else {
        break;
      }
    }
    /* 既知区間走行 */
    if (path.size()) {
      /* 既知区間斜めパターンに変換 */
      path =
          MazeLib::RobotBase::pathConvertSearchToKnown(path, rp.diag_enabled);
      /* 既知区間走行 */
      float straight = 0;
      for (int path_index = 0; path_index < path.length(); path_index++) {
        const auto action =
            static_cast<const MazeLib::RobotBase::FastAction>(path[path_index]);
        fast_run_switch(action, straight, rp);
      }
      /* 最後の直線を消化 */
      if (straight > 0.1f) {
        straight_x(straight, rp.max_speed, v_search, rp);
        straight = 0;
      }
    }
  }
  void search_run_task() {
    const auto &rp = rp_search;
    /* スタート */
    offset =
        ctrl::Position(field::SegWidthFull / 2,
                       field::SegWidthFull / 2 + model::CenterShift, M_PI / 2);
    sc.enable();
    while (1) {
      /* 壁を確認 */
      is_wall = wd.is_wall;
      // bz.play(Buzzer::SHORT7);
      /* 探索器に終了を通知 */
      if (q.empty())
        isRunningFlag = false;
      /* Actionがキューされるまで直進で待つ */
      queue_wait_decel(rp);
      /* 既知区間走行 */
      if (q.size() >= 2)
        search_run_known(rp);
      /* 探索走行 */
      if (!q.empty()) {
        const auto action = q.front();
        q.pop();
        search_run_switch(action, rp);
      }
    }
  }
  void search_run_switch(const RobotBase::Action action,
                         const RunParameter &rp) {
    const bool no_front_front_wall =
        tof.getDistance() > field::SegWidthFull * 2 + field::SegWidthFull / 2;
    const bool unknown_accel = rp.unknown_accel_enabled &&
                               continue_straight_if_no_front_wall &&
                               no_front_front_wall;
    const auto v_end = unknown_accel ? unknown_accel_velocity : v_search;
    switch (action) {
    case RobotBase::Action::START_STEP:
      imu.angle = 0;
      sc.position.clear();
      sc.position.x = model::TailLength + field::WallThickness / 2;
      offset = ctrl::Position(field::SegWidthFull / 2, 0, M_PI / 2);
      straight_x(field::SegWidthFull, v_end, v_end, rp);
      break;
    case RobotBase::Action::START_INIT:
      start_init();
      break;
    case RobotBase::Action::ST_FULL:
      if (tof.getDistance() < field::SegWidthFull)
        wall_stop();
      front_wall_fix(rp, 2 * field::SegWidthFull);
      straight_x(field::SegWidthFull, v_end, v_end, rp);
      break;
    case RobotBase::Action::ST_HALF:
      straight_x(field::SegWidthFull / 2 - model::CenterShift, v_search,
                 v_search, rp);
      break;
    case RobotBase::Action::TURN_L:
      front_wall_fix(rp, field::SegWidthFull);
      front_wall_fix(rp, 2 * field::SegWidthFull);
      if (sc.position.x < 5.0f && sc.ref_v.tra < v_search * 1.2f) {
        ctrl::slalom::Trajectory st(SS_S90L);
        straight_x(st.getShape().straight_prev, v_search, v_search, rp);
        if (wd.is_wall[0])
          wall_stop();
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
    case RobotBase::Action::TURN_R:
      front_wall_fix(rp, field::SegWidthFull);
      front_wall_fix(rp, 2 * field::SegWidthFull);
      if (sc.position.x < 5.0f && sc.ref_v.tra < v_search * 1.2f) {
        ctrl::slalom::Trajectory st(SS_S90R);
        straight_x(st.getShape().straight_prev, v_search, v_search, rp);
        if (wd.is_wall[1])
          wall_stop();
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
    case RobotBase::Action::ROTATE_180:
      uturn();
      break;
    case RobotBase::Action::ST_HALF_STOP:
      front_wall_fix(rp, field::SegWidthFull);
      front_wall_fix(rp, 2 * field::SegWidthFull);
      straight_x(field::SegWidthFull / 2 + model::CenterShift, v_search, 0, rp);
      turn(0); //*< 姿勢を整える */
      break;
    }
  }
  void fast_run_task() {
    /* パラメータを取得 */
    const auto &rp = rp_fast;
    /* 最短走行用にパターンを置換 */
    path = MazeLib::RobotBase::pathConvertSearchToFast(path, rp.diag_enabled);
    /* キャリブレーション */
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    /* 壁に背中を確実につける */
    mt.drive(-0.25f, -0.25f);
    delay(200);
    mt.free();
    /* 走行開始 */
    fan.drive(rp.fan_duty);
    delay(500);  //< ファンの回転数が一定なるのを待つ
    sc.enable(); //< 速度コントローラ始動
    /* 初期位置を設定 */
    offset =
        ctrl::Position(field::SegWidthFull / 2,
                       model::TailLength + field::WallThickness / 2, M_PI / 2);
    sc.position.clear();
    /* 最初の直線を追加 */
    float straight =
        field::SegWidthFull / 2 - model::TailLength - field::WallThickness / 2;
    /* 走行 */
    for (int path_index = 0; path_index < path.length(); path_index++) {
      const auto action =
          static_cast<const MazeLib::RobotBase::FastAction>(path[path_index]);
      fast_run_switch(action, straight, rp);
    }
    /* 最後の直線を消化 */
    if (straight > 0.1f) {
      straight_x(straight, rp.max_speed, 0, rp);
      straight = 0;
    }
    sc.set_target(0, 0);
    fan.drive(0);
    delay(200);
    sc.disable();
    bz.play(Buzzer::COMPLETE);
    path = "";
    isRunningFlag = false;
    vTaskDelay(portMAX_DELAY);
  }
  void fast_run_switch(const MazeLib::RobotBase::FastAction action,
                       float &straight, const RunParameter &rp) {
    switch (action) {
    case MazeLib::RobotBase::FastAction::FL45:
      SlalomProcess(SS_F45L, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR45:
      SlalomProcess(SS_F45R, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL45P:
      SlalomProcess(SS_F45L, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR45P:
      SlalomProcess(SS_F45R, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FLV90:
      SlalomProcess(SS_FV90L, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FRV90:
      SlalomProcess(SS_FV90R, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FLS90:
      SlalomProcess(SS_FS90L, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FRS90:
      SlalomProcess(SS_FS90R, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL90:
      SlalomProcess(SS_F90L, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR90:
      SlalomProcess(SS_F90R, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL135:
      SlalomProcess(SS_F135L, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR135:
      SlalomProcess(SS_F135R, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL135P:
      SlalomProcess(SS_F135L, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR135P:
      SlalomProcess(SS_F135R, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL180:
      SlalomProcess(SS_F180L, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR180:
      SlalomProcess(SS_F180R, straight, false, rp);
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
};
