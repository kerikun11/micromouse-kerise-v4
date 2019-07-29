#pragma once

#include "config/model.h"
#include "global.h"

#include "TrajectoryTracker.h"
#include "slalom_shapes.h"

#include "TaskBase.h"
#include <AccelDesigner.h>
#include <cmath>
#include <memory>
#include <queue>
#include <vector>

#define SEARCH_WALL_ATTACH_ENABLED 1
#define SEARCH_WALL_CUT_ENABLED 0
#define SEARCH_WALL_FRONT_ENABLED 1
#define SEARCH_WALL_AVOID_ENABLED 1

#define SEARCH_RUN_TASK_PRIORITY 3
#define SEARCH_RUN_STACK_SIZE 8192

#define SEARCH_RUN_VELOCITY 270.0f
#define SEARCH_RUN_V_MAX 600.0f

#include <RobotBase.h>
using namespace MazeLib;

class SearchRun : TaskBase {
public:
  struct RunParameter {
    RunParameter(const float curve_gain = 0.8, const float max_speed = 600,
                 const float accel = 3000)
        : curve_gain(curve_gain), max_speed(max_speed), accel(accel) {}
    float curve_gain;
    float max_speed;
    float accel;
    const float cg_gain = 1.05f;
    const float ms_gain = 1.2f;
    const float ac_gain = 1.1f;
    float search_v = SEARCH_RUN_VELOCITY;
    float search_v_max = SEARCH_RUN_V_MAX;
    bool wallAvoidFlag = true;
    bool wallAvoid45Flag = true;
    bool wallCutFlag = true;
    bool V90Enabled = true;
    bool KnownDiagEnabled = true;
    float fanDuty = 0.4f;
    static float getCurveGains(const int value) {
      float vals_p[] = {1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f, 1.7f};
      float vals_n[] = {1.0f, 0.9f, 0.8f, 0.7f, 0.6f, 0.5f, 0.4f, 0.3f, 0.2f};
      return value > 0 ? vals_p[value] : vals_n[-value];
    }
    static float getMaxSpeeds(const int value) {
      float vals_p[] = {600, 720, 840, 960, 1080, 1200};
      float vals_n[] = {600, 480, 360, 240, 180, 120, 60};
      return value > 0 ? vals_p[value] : vals_n[-value];
    }
    static float getAccels(const int value) {
      float vals_p[] = {4800, 6000, 7200, 8400, 9600, 10800};
      float vals_n[] = {4800, 3600, 2400, 1200, 600, 600};
      return value > 0 ? vals_p[value] : vals_n[-value];
    }
  };
#ifndef M_PI
  static constexpr float M_PI = 3.14159265358979323846f;
#endif

public:
  SearchRun() {}
  virtual ~SearchRun() {}
  void enable() {
    deleteTask();
    createTask("SearchRun", SEARCH_RUN_TASK_PRIORITY, SEARCH_RUN_STACK_SIZE);
  }
  void disable() {
    deleteTask();
    sc.disable();
    while (q.size()) {
      q.pop();
    }
  }
  void set_action(enum RobotBase::Action action) {
    q.push(action);
    isRunningFlag = true;
  }
  bool isRunning() { return isRunningFlag; }
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
    float min_dist = 999;
    int min_index = 0;
    for (int i = 0; i < table_size; ++i) {
      if (min_dist > table[i]) {
        min_dist = table[i];
        min_index = i;
      }
    }
    sc.position.clear();
    turn(2 * M_PI * min_index / table_size);
    sc.position.clear();
    /* 前壁補正 */
    wall_attach(true);
    turn(wd.distance.side[0] > wd.distance.side[1] ? M_PI / 2 : -M_PI / 2);
    wall_attach(true);
    delay(50);
#if 0
    for (int i = 0; i < 4; ++i) {
      wall_attach(true);
      turn(M_PI / 2);
    }
#endif
    while (1) {
      if (!wd.wall[2])
        break;
      wall_attach();
      turn(-M_PI / 2);
    }
    sc.disable();
    return true;
  }

public:
  RunParameter rp;

private:
  std::queue<enum RobotBase::Action> q;
  bool isRunningFlag = false;
  Position offset;
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
    if ((force && tof.getDistance() < 180) || tof.getDistance() < 90 ||
        (wd.distance.front[0] > 0 && wd.distance.front[1] > 0)) {
      led = 6;
      bz.play(Buzzer::SHORT);
      tof.disable();
      delay(10);
      portTickType xLastWakeTime = xTaskGetTickCount();
      WheelParameter wi;
      for (int i = 0; i < 2000; i++) {
        const float Kp = 120.0f;
        const float Ki = 0.5f;
        const float sat_integral = 1.0f;
        const float end = 0.05f;
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
        const float sat_tra = 180.0f;   //< [mm/s]
        const float sat_rot = M_PI / 2; //< [rad/s]
        sc.set_target(saturate(wp.tra, sat_tra), saturate(wp.rot, sat_rot));
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      }
      sc.set_target(0, 0);
      sc.position.x = 0;  //< 直進方向の補正
      sc.position.th = 0; //< 回転方向の補正
      tof.enable();
      // bz.play(Buzzer::SHORT);
      led = 0;
    }
#endif
  }
  void wall_avoid(const float remain) {
#if SEARCH_WALL_AVOID_ENABLED
    /* 一定速より小さかったら行わない */
    if (sc.est_v.tra < 100.0f)
      return;
    /* 曲線なら前半しか行わない */
    if (std::abs(sc.position.th) > M_PI * 0.1f)
      return;
    uint8_t led_flags = 0;
    /* 90 [deg] の倍数 */
    if (rp.wallAvoidFlag && isAlong()) {
      const float gain = 0.01f;
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
    if (rp.wallAvoid45Flag && isDiag() && remain > field::SegWidthFull / 3) {
      const float shift = 0.04f;
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
    // if (velocity < 120)
    //   return;
    // for (int i = 0; i < 2; i++) {
    //   const float normal_th = sc.position.th / (M_PI / 2);
    //   const float frac_part = normal_th - roundf(normal_th);
    //   const float diff_thr = velocity * 2.0f;
    //   if (wd.diff.side[i] < -diff_thr && std::abs(frac_part) < 0.01f) {
    //     bz.play(Buzzer::SHORT);
    //     const float wall_cut_x = -18;
    //     auto th = roundf(sc.position.th / (M_PI / 2)) * M_PI / 2;
    //     auto offset_x = offset.rotate(-offset.th - th).x;
    //     auto fixed = sc.position.rotate(-th);
    //     auto x = offset_x + fixed.x;
    //     auto fixed_x = roundf((x - wall_cut_x) / 90) * 90 + wall_cut_x;
    //     fixed.x = fixed_x - offset_x;
    //     sc.position = fixed.rotate(th);
    //   }
    // }
    if (!wallCutFlag)
      return;
    if (!V90Enabled)
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
            bz.play(Buzzer::SHORT);
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
            bz.play(Buzzer::SHORT);
          }
        }
      }
      prev_wall[i] = wd.wall[i];
    }
#endif
  }
  void wall_calib(const float velocity) {
#if SEARCH_WALL_FRONT_ENABLED
    if (wd.wall[2] && tof.passedTimeMs() < 100) {
      float value =
          tof.getDistance() - (5 + tof.passedTimeMs()) / 1000.0f * velocity;
      // value = value / std::cos(sc.position.th); /*< 機体姿勢考慮 */
      if (value > 60 && value < 120) {
        const float fixed_x = 90 - value + 5;
        if (fixed_x < 10) {
          sc.position.x = fixed_x;
          // bz.play(Buzzer::SHORT);
        }
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
    offset += Position(0, 0, angle).rotate(offset.th);
  }
  void straight_x(const float distance, const float v_max, const float v_end) {
    if (distance - sc.position.x > 0) {
      const float jerk = 500000;
      const float accel = 3000;
      const float v_start = sc.ref_v.tra;
      TrajectoryTracker tt(model::tt_gain);
      tt.reset(v_start);
      AccelDesigner ad(jerk, accel, v_start, v_max, v_end,
                       distance - sc.position.x);
      float int_y = 0;
      portTickType xLastWakeTime = xTaskGetTickCount();
      for (float t = 0; t < ad.t_end(); t += 0.001f) {
        auto est_q = sc.position;
        auto ref_q = Position(ad.x(t), 0);
        auto ref_dq = Position(ad.v(t), 0);
        auto ref_ddq = Position(ad.a(t), 0);
        auto ref_dddq = Position(ad.j(t), 0);
        auto ref = tt.update(est_q, sc.est_v, sc.est_a, ref_q, ref_dq, ref_ddq,
                             ref_dddq);
        sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        const float remain = distance - est_q.x;
        wall_avoid(remain);
        wall_cut(ref.v);
        /* 機体姿勢の補正 */
        int_y += sc.position.y;
        sc.position.th += int_y * 0.00000001f;
      }
    }
    if (v_end < 1.0f)
      sc.set_target(0, 0);
    sc.position.x -= distance; //< 移動した量だけ位置を更新
    offset += Position(distance, 0, 0).rotate(offset.th);
  }
  void trace(slalom::Trajectory &sd, const float velocity) {
    TrajectoryTracker tt(model::tt_gain);
    tt.reset(velocity);
    slalom::State s;
    const float Ts = 0.001f;
    sd.reset(velocity);
    portTickType xLastWakeTime = xTaskGetTickCount();
    for (float t = 0; t < sd.t_end(); t += 0.001f) {
      sd.update(&s, Ts);
      auto est_q = sc.position;
      auto ref = tt.update(est_q, sc.est_v, sc.est_a, s.q, s.dq, s.ddq, s.dddq);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      wall_avoid(0);
      wall_cut(ref.v);
    }
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
    if (straight > 1.0f) {
      straight_x(straight, rp.max_speed, velocity);
      straight = 0;
    }
    trace(st, velocity);
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
  void stop() {
    bz.play(Buzzer::ERROR);
    float v = sc.est_v.tra;
    while (v > 0) {
      sc.set_target(v, 0);
      v -= 9;
      delay(1);
    }
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
    while (q.empty()) {
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      if (v > 0)
        v -= 6;
      xLastWakeTime = xTaskGetTickCount();
      Position cur = sc.position;
      float th = atan2f(-cur.y, (2 + 20 * v / 240)) - cur.th;
      sc.set_target(v, 40 * th);
    }
  }
  void task() override {
    /* スタート */
    sc.enable();
    while (1) {
      if (q.empty())
        isRunningFlag = false;
      /* Actionがキューされるまで直進で待つ */
      queue_wait_decel();
      /* キューを消化 */
      if (rp.KnownDiagEnabled && q.size() >= 2) {
        std::string path;
        while (1) {
          if (q.empty())
            break;
          const auto action = q.front();
          switch (action) {
          case RobotBase::Action::ST_HALF:
            path += RobotBase::FastAction::F_ST_HALF;
            break;
          case RobotBase::Action::ST_FULL:
            path += RobotBase::FastAction::F_ST_FULL;
            break;
          case RobotBase::Action::TURN_L:
            path += RobotBase::FastAction::FLS90;
            break;
          case RobotBase::Action::TURN_R:
            path += RobotBase::FastAction::FRS90;
            break;
          default:
            goto known_diag_break;
          }
          q.pop();
        }
      known_diag_break:
        if (path.size()) {
          path = MazeLib::RobotBase::pathConvertSearchToKnown(path);
          float straight = 0;
          for (int path_index = 0; path_index < path.length(); path_index++) {
            printf("FastRun: %c, st => %.1f\n", path[path_index], straight);
            const auto action =
                static_cast<const MazeLib::RobotBase::FastAction>(
                    path[path_index]);
            fast_run_switch(action, straight, rp);
          }
          /* 最後の直線を消化 */
          if (straight > 1.0f) {
            straight_x(straight, rp.max_speed, rp.search_v);
            straight = 0;
          }
        }
      }
      /* 探索走行 */
      if (q.size()) {
        const auto action = q.front();
        q.pop();
        int num = 1;
        while (!q.empty()) {
          auto next = q.front();
          if (action != RobotBase::Action::ST_FULL || action != next)
            break;
          num++;
          q.pop();
        }
        search_run_switch(action, rp, num);
      }
    }
    vTaskDelay(portMAX_DELAY);
  }
  void search_run_switch(const RobotBase::Action action, RunParameter rp,
                         const int num = 1) {
    const float velocity = rp.search_v;
    const float v_max = rp.search_v_max;
    switch (action) {
    case RobotBase::Action::START_STEP:
      sc.position.clear();
      imu.angle = 0;
      offset = Position(field::SegWidthFull / 2,
                        model::TailLength + field::WallThickness / 2, M_PI / 2);
      straight_x(field::SegWidthFull - model::TailLength -
                     field::WallThickness / 2,
                 velocity, velocity);
      break;
    case RobotBase::Action::START_INIT:
      straight_x(field::SegWidthFull / 2 + model::CenterShift, velocity, 0);
      start_init();
      break;
    case RobotBase::Action::ST_FULL:
      if (wd.wall[2])
        stop();
      straight_x(field::SegWidthFull * num, num > 1 ? v_max : velocity,
                 velocity);
      break;
    case RobotBase::Action::ST_HALF:
      straight_x(field::SegWidthFull / 2 * num - model::CenterShift, velocity,
                 velocity);
      break;
    case RobotBase::Action::TURN_L: {
      if (wd.wall[0])
        stop();
      wall_calib(velocity);
      slalom::Trajectory st(SS_SL90);
      straight_x(st.get_straight_prev(), velocity, velocity);
      trace(st, velocity);
      straight_x(st.get_straight_post(), velocity, velocity);
      break;
    }
    case RobotBase::Action::TURN_R: {
      if (wd.wall[1])
        stop();
      wall_calib(velocity);
      slalom::Trajectory st(SS_SR90);
      straight_x(st.get_straight_prev(), velocity, velocity);
      trace(st, velocity);
      straight_x(st.get_straight_post(), velocity, velocity);
      break;
    }
    case RobotBase::Action::ROTATE_180:
      uturn();
      break;
    case RobotBase::Action::ST_HALF_STOP:
      straight_x(field::SegWidthFull / 2 + model::CenterShift, velocity, 0);
      turn(0);
      sc.disable();
      isRunningFlag = false;
      vTaskDelay(portMAX_DELAY);
      break;
    }
  }
  void fast_run_switch(const MazeLib::RobotBase::FastAction action,
                       float &straight, const RunParameter runParameter) {
    switch (action) {
    case MazeLib::RobotBase::FastAction::FL45:
      SlalomProcess(SS_FL45, straight, false, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FR45:
      SlalomProcess(SS_FR45, straight, false, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FL45P:
      SlalomProcess(SS_FL45, straight, true, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FR45P:
      SlalomProcess(SS_FR45, straight, true, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FLV90:
      SlalomProcess(SS_FLV90, straight, false, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FRV90:
      SlalomProcess(SS_FRV90, straight, false, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FLS90:
      SlalomProcess(SS_FLS90, straight, false, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FRS90:
      SlalomProcess(SS_FRS90, straight, false, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FL90:
      SlalomProcess(SS_FL90, straight, false, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FR90:
      SlalomProcess(SS_FR90, straight, false, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FL135:
      SlalomProcess(SS_FL135, straight, false, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FR135:
      SlalomProcess(SS_FR135, straight, false, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FL135P:
      SlalomProcess(SS_FL135, straight, true, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FR135P:
      SlalomProcess(SS_FR135, straight, true, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FL180:
      SlalomProcess(SS_FL180, straight, false, runParameter);
      break;
    case MazeLib::RobotBase::FastAction::FR180:
      SlalomProcess(SS_FR180, straight, false, runParameter);
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
