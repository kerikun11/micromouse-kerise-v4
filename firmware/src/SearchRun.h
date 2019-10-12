#pragma once

#include "config/model.h"
#include "global.h"

#include "TrajectoryTracker.h"
#include "slalom_shapes.h"

#include "TaskBase.h"
#include <AccelDesigner.h>
#include <cmath>
#include <queue>
#include <vector>

#define SEARCH_WALL_ATTACH_ENABLED 1
#define SEARCH_WALL_CUT_ENABLED 0
#define SEARCH_WALL_FRONT_ENABLED 1
#define SEARCH_WALL_AVOID_ENABLED 1
#define SEARCH_WALL_THETA_ENABLED 1
#define SEARCH_NO_FRONT_WALL_ACCEL_ENABLED 1

#define SEARCH_RUN_TASK_PRIORITY 3
#define SEARCH_RUN_STACK_SIZE 8192

#include <RobotBase.h>
using namespace MazeLib;

class SearchRun : TaskBase {
public:
  struct RunParameter {
  public:
#if KERISE_SELECT == 4
    float search_v = 300;
    float curve_gain = 1.1;
    float max_speed = 720;
    float accel = 3600;
#else
    float search_v = 240;
    float curve_gain = 1.0;
    float max_speed = 480;
    float accel = 2400;
#endif
    float fan_duty = 0.2f;
    bool diag_enabled = 1;
    bool front_wall_fix_enabled = 1;
    bool wall_avoid_enabled = 1;

  public:
    static constexpr float cg_gain = 1.05f;
    static constexpr float ms_gain = 1.21f;
    static constexpr float ac_gain = 1.1f;
  };
#ifndef M_PI
  static constexpr float M_PI = 3.14159265358979323846f;
#endif

public:
  SearchRun() {}
  ~SearchRun() {}
  void enable() {
    deleteTask();
    isRunningFlag = true;
    createTask("SearchRun", SEARCH_RUN_TASK_PRIORITY, SEARCH_RUN_STACK_SIZE);
  }
  void disable() {
    deleteTask();
    sc.disable();
    while (q.size())
      q.pop();
    path = "";
    isRunningFlag = false;
  }
  void set_action(RobotBase::Action action) {
    q.push(action);
    isRunningFlag = true;
  }
  void set_path(std::string path) { this->path = path; }
  bool isRunning() { return isRunningFlag; }

public:
  RunParameter rp_search;
  RunParameter rp_fast;
  bool continue_straight_if_no_front_wall = false;

  bool positionRecovery() {
    sc.enable();
    static constexpr float m_dddth = 4800 * M_PI;
    static constexpr float m_ddth = 48 * M_PI;
    static constexpr float m_dth = 2 * M_PI;
    const float angle = 2 * M_PI;
    constexpr int table_size = 180;
    std::array<float, table_size> table;
    for (auto &t : table)
      t = 255;
    AccelDesigner ad(m_dddth, m_ddth, 0, m_dth, 0, angle);
    portTickType xLastWakeTime = xTaskGetTickCount();
    const float back_gain = 10.0f;
    int index = 0;
    for (float t = 0; t < ad.t_end(); t += 0.001f) {
      float delta = sc.position.x * std::cos(-sc.position.th) -
                    sc.position.y * std::sin(-sc.position.th);
      sc.set_target(-delta * back_gain, ad.v(t), 0, ad.a(t));
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      if (ad.x(t) > 2 * PI * index / table_size) {
        index++;
        table[index % table_size] = tof.getDistance();
      }
    }
    /* 最小分散を探す */
    float min_var = 999;
    int min_i = 0;
    for (int i = 0; i < table_size; ++i) {
      const int window = table_size / 6;
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
    sc.enable(); //< reset
    turn(2 * M_PI * min_i / table_size);
    /* 前壁補正 */
    wall_attach(true);
    turn(wd.distance.side[0] > wd.distance.side[1] ? M_PI / 2 : -M_PI / 2);
    // wall_attach(true);
    delay(50); //< ToFが有効化するのを待つ
    while (1) {
      if (!wd.wall[2])
        break;
      wall_attach();
      turn(-M_PI / 2);
    }
    sc.disable();
    return true;
  }

private:
  std::queue<RobotBase::Action> q;
  bool isRunningFlag = false;
  ctrl::TrajectoryTracker tt{model::tt_gain};
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
    if ((force && tof.getDistance() < 210) || tof.getDistance() < 90 ||
        (wd.distance.front[0] > 0 && wd.distance.front[1] > 0)) {
      led = 6;
      bz.play(Buzzer::SHORT6);
      tof.disable();
      delay(10);
      portTickType xLastWakeTime = xTaskGetTickCount();
      WheelParameter wi;
      for (int i = 0; i < 2000; i++) {
        const float Kp = model::wall_attach_gain_Kp;
        const float Ki = model::wall_attach_gain_Ki;
        const float sat_integral = 60.0f;
        const float end = 1.0f;
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
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
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
  void wall_avoid(const float remain, const RunParameter &rp) {
#if SEARCH_WALL_AVOID_ENABLED
    /* 一定速より小さかったら行わない */
    if (!rp.wall_avoid_enabled || sc.est_v.tra < 150.0f)
      return;
    /* 曲線なら前半しか行わない */
    if (std::abs(sc.position.th) > M_PI * 0.1f)
      return;
    uint8_t led_flags = 0;
    /* 90 [deg] の倍数 */
    if (isAlong()) {
      const float gain = 0.006f;
      const float wall_diff_thr = 100;
      if (wd.wall[0] && std::abs(wd.diff.side[0]) < wall_diff_thr) {
        sc.position.y += wd.distance.side[0] * gain;
        led_flags |= 8;
      }
      if (wd.wall[1] && std::abs(wd.diff.side[1]) < wall_diff_thr) {
        sc.position.y -= wd.distance.side[1] * gain;
        led_flags |= 1;
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
#endif
  }
  void wall_cut(const float velocity) {
#if SEARCH_WALL_CUT_ENABLED
    if (!wallCutFlag)
      return;
    if (!diag_enabled)
      return;
    if (velocity < 120)
      return;
    /* 曲線なら前半しか使わない */
    if (std::abs(sc.position.th) > M_PI * 0.1f)
      return;
    for (int i = 0; i < 2; i++) {
      if (prev_wall[i] && !wd.wall[i]) {
        /* 90 [deg] の倍数 */
        if (isAlong()) {
          Position prev = sc.position;
          Position fix = sc.position.rotate(-origin.th);
          const float wall_cut_offset = -15; /*< from wall_cut_line */
          fix.x = round2(fix.x, field::SegWidthFull) + wall_cut_offset;
          fix = fix.rotate(origin.th);
          const float diff =
              fix.rotate(-origin.th).x - prev.rotate(-origin.th).x;
          if (-30 < diff && diff < 5) {
            sc.position = fix;
            bz.play(Buzzer::SHORT6);
          }
        }
        /* 45 [deg] + 90 [deg] の倍数 */
        if (isDiag()) {
          Position prev = sc.position;
          Position fix = sc.position.rotate(-origin.th);
          const float extra = 31;
          if (i == 0) {
            fix.x = round2(fix.x - extra - field::SegWidthDiag / 2,
                           field::SegWidthDiag) +
                    field::SegWidthDiag / 2 + extra;
          } else {
            fix.x = round2(fix.x - extra, field::SegWidthDiag) + extra;
          }
          fix = fix.rotate(origin.th);
          if (fabs(prev.rotate(-origin.th).x - fix.rotate(-origin.th).x) <
              10.0f) {
            // sc.position = fix;
            bz.play(Buzzer::SHORT6);
          }
        }
      }
      prev_wall[i] = wd.wall[i];
    }
#endif
  }
  void wall_front_fix(const RunParameter rp, const float dist_to_wall) {
#if SEARCH_WALL_FRONT_ENABLED
    if (rp.front_wall_fix_enabled && tof.isValid()) {
      /* 壁までの距離を推定 */
      float value =
          tof.getDistance() - (tof.passedTimeMs() + 5) / 1000.0f * sc.ref_v.tra;
      // tof.getDistance() - tof.passedTimeMs() / 1000.0f * rp.search_v;
      value = value * std::cos(sc.position.th);  /*< 機体姿勢考慮 */
      float fixed_x = dist_to_wall - value + 12; /*< 大きく:壁に近く */
      const float max_x = 5;
      if (-30 < fixed_x && fixed_x < 30) {
        if (fixed_x > max_x) {
          fixed_x = max_x;
          // bz.play(Buzzer::SHORT7);
        }
        sc.position.x = fixed_x;
        // bz.play(Buzzer::SHORT7);
      }
    }
#endif
  }
  void turn(const float angle) {
    static constexpr float m_dddth = 4800 * M_PI;
    static constexpr float m_ddth = 48 * M_PI;
    static constexpr float m_dth = 4 * M_PI;
    AccelDesigner ad(m_dddth, m_ddth, 0, m_dth, 0, angle);
    portTickType xLastWakeTime = xTaskGetTickCount();
    const float back_gain = 10.0f;
    for (float t = 0; t < ad.t_end(); t += 0.001f) {
      float delta = sc.position.x * std::cos(-sc.position.th) -
                    sc.position.y * std::sin(-sc.position.th);
      sc.set_target(-delta * back_gain, ad.v(t), 0, ad.a(t));
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    }
    float int_error = 0;
    while (1) {
      float delta = sc.position.x * std::cos(-sc.position.th) -
                    sc.position.y * std::sin(-sc.position.th);
      const float Kp = 20.0f;
      const float Ki = 10.0f;
      const float error = angle - sc.position.th;
      int_error += error * 0.001f;
      sc.set_target(-delta * back_gain, Kp * error + Ki * int_error);
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      if (std::abs(Kp * error) + std::abs(Ki * int_error) < 0.05f * PI)
        break;
    }
    sc.set_target(0, 0);
    sc.position.th -= angle; //< 移動した量だけ位置を更新
    sc.position = sc.position.rotate(-angle); //< 移動した量だけ位置を更新
    offset += ctrl::Position(0, 0, angle).rotate(offset.th);
  }
  void straight_x(const float distance, const float v_max, const float v_end,
                  const RunParameter &rp) {
    if (distance - sc.position.x > 0) {
      const float jerk = 240000;
      const float v_start = sc.ref_v.tra;
      AccelDesigner ad(jerk, rp.accel, v_start, v_max, v_end,
                       distance - sc.position.x, sc.position.x);
      float int_y = 0;
      portTickType xLastWakeTime = xTaskGetTickCount();
      for (float t = 0; t < ad.t_end(); t += 0.001f) {
        auto est_q = sc.position;
        auto ref_q = ctrl::Position(ad.x(t), 0);
        auto ref_dq = ctrl::Position(ad.v(t), 0);
        auto ref_ddq = ctrl::Position(ad.a(t), 0);
        auto ref_dddq = ctrl::Position(ad.j(t), 0);
        auto ref = tt.update(est_q, sc.est_v, sc.est_a, ref_q, ref_dq, ref_ddq,
                             ref_dddq);
        sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        const float remain = distance - est_q.x;
        wall_avoid(remain, rp);
        wall_cut(ref.v);
        /* 機体姿勢の補正 */
        int_y += sc.position.y;
#if SEARCH_WALL_THETA_ENABLED
        sc.position.th += int_y * 0.00000001f;
#endif
      }
    }
    sc.position.x -= distance; //< 移動した量だけ位置を更新
    offset += ctrl::Position(distance, 0, 0).rotate(offset.th);
  }
  void trace(slalom::Trajectory &sd, const float velocity,
             const RunParameter &rp) {
    slalom::State s;
    const float Ts = 0.001f;
    sd.reset(velocity);
    portTickType xLastWakeTime = xTaskGetTickCount();
#if SEARCH_WALL_FRONT_ENABLED
    float front_fix_x = 0;
#endif
    s.q.x = sc.position.x; //*< 既に移動した分を反映 */
    for (float t = 0; t < sd.t_end(); t += 0.001f) {
      sd.update(&s, Ts);
      auto est_q = sc.position;
      auto ref = tt.update(est_q, sc.est_v, sc.est_a, s.q, s.dq, s.ddq, s.dddq);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      wall_avoid(0, rp);
      wall_cut(ref.v);
#if SEARCH_WALL_FRONT_ENABLED
      /* V90ターン中の前壁補正 */
      if (tof.isValid() && std::abs(t - sd.t_end() / 2) < 0.0005001f &&
          (sd.getShape() == SS_FLV90 || sd.getShape() == SS_FRV90)) {
        float tof_value =
            tof.getDistance() - tof.passedTimeMs() / 1000.0f * velocity;
        float fixed_x =
            field::SegWidthFull - tof_value + 12; /*< 要調整, 大きく:前壁近く*/
        if (-20 < fixed_x && fixed_x < 20) {
          front_fix_x = fixed_x;
          // bz.play(Buzzer::SHORT7);
        }
      }
#endif
    }
#if SEARCH_WALL_FRONT_ENABLED
    /* V90ターン中の前壁補正 */
    if (rp.front_wall_fix_enabled && front_fix_x != 0)
      sc.position +=
          ctrl::Position(front_fix_x, 0, 0).rotate(sd.get_net_curve().th / 2);
#endif
    sc.set_target(velocity, 0);
    const auto net = sd.get_net_curve();
    sc.position = (sc.position - net).rotate(-net.th);
    offset += net.rotate(offset.th);
  }
  void SlalomProcess(const slalom::Shape &shape, float &straight,
                     const bool reverse, const RunParameter &rp) {
    slalom::Trajectory st(shape);
    const float velocity = st.get_v_ref() * rp.curve_gain;
    straight += !reverse ? st.get_straight_prev() : st.get_straight_post();
    /* ターン前の直線を消化 */
    if (straight > 1.0f) {
      straight_x(straight, rp.max_speed, velocity, rp);
      straight = 0;
    }
    /* 直線前壁補正 */
    if (isAlong()) {
      if (rp.diag_enabled && reverse == false) {
        wall_front_fix(rp, field::SegWidthFull + field::SegWidthFull / 2 -
                               st.get_straight_prev());
        wall_front_fix(rp, 2 * field::SegWidthFull + field::SegWidthFull / 2 -
                               st.get_straight_prev());
      }
      if (shape == SS_FLS90 || shape == SS_FRS90) {
        wall_front_fix(rp, field::SegWidthFull - st.get_straight_prev());
        wall_front_fix(rp, 2 * field::SegWidthFull - st.get_straight_prev());
        // 壁衝突防止
        if (shape.total.th > 0 ? wd.wall[0] : wd.wall[1])
          wall_stop();
      }
    }
    /* 斜め前壁補正 */
    // if (isDiag())
    //   wall_front_fix(velocity, field::SegWidthDiag - st.get_straight_prev());
    /* スラローム */
    trace(st, velocity, rp);
    straight += reverse ? st.get_straight_prev() : st.get_straight_post();
  }
  void put_back() {
    const int max_v = 150;
    const float th_gain = 100.0f;
    for (int i = 0; i < max_v; i++) {
      sc.set_target(-i, -sc.position.th * th_gain);
      delay(1);
    }
    for (int i = 0; i < 100; i++) {
      sc.set_target(-max_v, -sc.position.th * th_gain);
      delay(1);
    }
    sc.disable();
    mt.drive(-0.1f, -0.1f);
    delay(200);
    mt.drive(-0.2f, -0.2f);
    delay(200);
    sc.enable(true);
    tt.reset();
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
    bz.play(Buzzer::ERROR);
    float v = sc.est_v.tra;
    while (v > 0) {
      sc.set_target(v, 0);
      v -= 9;
      delay(1);
    }
    delay(100);
    sc.disable();
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
  void queue_wait_decel() {
    /* Actionがキューされるまで直進で待つ */
    float v = sc.ref_v.tra;
    portTickType xLastWakeTime = xTaskGetTickCount();
    for (int i = 0; q.empty(); ++i) {
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      if (v > 0)
        v -= 6;
      xLastWakeTime = xTaskGetTickCount();
      const auto cur = sc.position;
      float th = atan2f(-cur.y, (2 + 20 * v / 240)) - cur.th;
      sc.set_target(v, 40 * th);
      if (i == 60)
        bz.play(Buzzer::MAZE_BACKUP);
    }
    sc.position.y = 0; /**< 横のステップ変化を抑制 */
  }
  void task() override {
    if (q.size())
      search_run_task();
    else
      fast_run_task();
    vTaskDelay(portMAX_DELAY);
  }
  void search_run_known(const RunParameter &rp) {
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
        straight_x(straight, rp.max_speed, rp.search_v, rp);
        straight = 0;
      }
    }
  }
  void search_run_task() {
    const auto &rp = rp_search;
    /* スタート */
    offset = ctrl::Position(field::SegWidthFull / 2 + model::CenterShift,
                            field::SegWidthFull / 2, 0);
    sc.enable();
    tt.reset();
    while (1) {
      if (q.empty())
        isRunningFlag = false;
      /* Actionがキューされるまで直進で待つ */
      queue_wait_decel();
      /* 既知区間走行 */
      if (q.size() >= 2)
        search_run_known(rp);
      /* 探索走行 */
      if (q.size()) {
        const auto action = q.front();
        q.pop();
        search_run_switch(action, rp);
      }
    }
  }
  void search_run_switch(const RobotBase::Action action,
                         const RunParameter &rp) {
    const float velocity = rp.search_v;
#if SEARCH_NO_FRONT_WALL_ACCEL_ENABLED
    const bool no_front_wall =
        tof.getDistance() > field::SegWidthFull * 2 + field::SegWidthFull / 4;
    const auto v_end =
        (continue_straight_if_no_front_wall && no_front_wall) ? 600 : velocity;
#else
    const auto v_end = velocity;
#endif
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
    case RobotBase::Action::ST_FULL: {
      if (wd.wall[2])
        wall_stop();
      // wall_front_fix(rp, 2 * field::SegWidthFull);
      straight_x(field::SegWidthFull, v_end, v_end, rp);
      break;
    }
    case RobotBase::Action::ST_HALF:
      straight_x(field::SegWidthFull / 2 - model::CenterShift, velocity,
                 velocity, rp);
      break;
    case RobotBase::Action::TURN_L: {
      wall_front_fix(rp, field::SegWidthFull);
      // wall_front_fix(rp, 2 * field::SegWidthFull);
      slalom::Trajectory st(SS_SL90);
      straight_x(st.get_straight_prev(), velocity, velocity, rp);
      if (wd.wall[0])
        wall_stop();
      trace(st, velocity, rp);
      straight_x(st.get_straight_post(), velocity, velocity, rp);
      break;
    }
    case RobotBase::Action::TURN_R: {
      wall_front_fix(rp, field::SegWidthFull);
      // wall_front_fix(rp, 2 * field::SegWidthFull);
      slalom::Trajectory st(SS_SR90);
      straight_x(st.get_straight_prev(), velocity, velocity, rp);
      if (wd.wall[1])
        wall_stop();
      trace(st, velocity, rp);
      straight_x(st.get_straight_post(), velocity, velocity, rp);
      break;
    }
    case RobotBase::Action::ROTATE_180:
      uturn();
      break;
    case RobotBase::Action::ST_HALF_STOP:
      wall_front_fix(rp, field::SegWidthFull);
      // wall_front_fix(rp, 2 * field::SegWidthFull);
      straight_x(field::SegWidthFull / 2 + model::CenterShift, velocity, 0, rp);
      turn(0);
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
    mt.drive(-0.2f, -0.2f);
    delay(200);
    mt.free();
    /* 走行開始 */
    fan.drive(rp.fan_duty);
    delay(500);  //< ファンの回転数が一定なるのを待つ
    sc.enable(); //< 速度コントローラ始動
    tt.reset();
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
      SlalomProcess(SS_FL45, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR45:
      SlalomProcess(SS_FR45, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL45P:
      SlalomProcess(SS_FL45, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR45P:
      SlalomProcess(SS_FR45, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FLV90:
      SlalomProcess(SS_FLV90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FRV90:
      SlalomProcess(SS_FRV90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FLS90:
      SlalomProcess(SS_FLS90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FRS90:
      SlalomProcess(SS_FRS90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL90:
      SlalomProcess(SS_FL90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR90:
      SlalomProcess(SS_FR90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL135:
      SlalomProcess(SS_FL135, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR135:
      SlalomProcess(SS_FR135, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL135P:
      SlalomProcess(SS_FL135, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR135P:
      SlalomProcess(SS_FR135, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL180:
      SlalomProcess(SS_FL180, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR180:
      SlalomProcess(SS_FR180, straight, false, rp);
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
