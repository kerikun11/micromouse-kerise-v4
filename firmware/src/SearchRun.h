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

#define SEARCH_RUN_VELOCITY 240.0f
#define SEARCH_RUN_V_MAX 600.0f

#include <RobotBase.h>
using namespace MazeLib;

class SearchRun : TaskBase {
public:
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
#if 1
    {
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

      // float min_dist = 999;
      // int min_index = 0;
      // int width = table_size / 4;
      // for (int i = 0; i < table_size; ++i) {
      //   float sum = 0;
      //   for (int j = -width / 2; j < width / 2; ++j)
      //     sum += table[(table_size + i + j) % table_size];
      //   const float mean = sum / width;
      //   float var = 0;
      //   for (int j = -width / 2; j < width / 2; ++j)
      //     var += std::pow(table[(table_size + i + j) % table_size] - mean,
      //     2);
      //   var /= table_size;
      //   if (min_dist > var) {
      //     min_dist = var;
      //     min_index = i;
      //   }
      // }
      // sc.position.clear();
      // turn(2 * M_PI * min_index / table_size);
      // sc.position.clear();

      // float min_dist = 999;
      // int min_index = 0;
      // for (int i = 0; i < table_size; ++i) {
      //   if (min_dist > table[i]) {
      //     min_dist = table[i];
      //     min_index = i;
      //   }
      // }
      // sc.position.clear();
      // turn(2 * M_PI * min_index / table_size);
      // sc.position.clear();
      // const float rho_max = 180;
      // float map[table_size][table_size] = {0};
      //   const auto p_map = std::make_unique<
      //       std::array<std::array<int16_t, table_size>, table_size>>();
      //   auto &map = *p_map;
      //   for (auto &a : map)
      //     for (auto &b : a)
      //       b = 0;
      //   for (int i = 0; i < table_size; ++i) {
      //     const auto x = table[i] * std::cos(2 * M_PI * i / table_size);
      //     const auto y = table[i] * std::sin(2 * M_PI * i / table_size);
      //     for (int j = 0; j < table_size; ++j) {
      //       const float th = 2 * M_PI * j / table_size;
      //       const float rho = x * std::cos(th) + y * std::sin(th);
      //       int rho_index = rho / rho_max * table_size;
      //       rho_index = std::min(rho_index, table_size - 1);
      //       if (rho_index >= 0)
      //         map[j][rho_index]++;
      //     }
      //   }
      //   int max_th = 0;
      //   int max_rho = 0;
      //   int max_map = 0;
      //   for (int i = 0; i < table_size; ++i)
      //     for (int j = 0; j < table_size; ++j)
      //       if (max_map < map[i][j]) {
      //         max_map = map[i][j];
      //         max_th = i;
      //         max_rho = j;
      //       }
      //   sc.position.clear();
      //   turn(2 * M_PI * max_th / table_size);
      //   sc.position.clear();
    }
    wall_attach(true);
    turn(wd.distance.side[0] > wd.distance.side[1] ? M_PI / 2 : -M_PI / 2);
    wall_attach(true);
    delay(50);
#endif
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

private:
  std::queue<enum RobotBase::Action> q;
  volatile bool isRunningFlag = false;
  Position offset;

  static auto saturate(auto src, auto sat) {
    return std::max(std::min(src, sat), -sat);
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
  void wall_avoid() {
#if SEARCH_WALL_AVOID_ENABLED
    if (std::abs(sc.position.th) < 0.05 * PI) {
      const float gain = 0.006;
      const float diff_thr = 100;
      if (wd.wall[0] && std::abs(wd.diff.side[0]) < diff_thr)
        sc.position.y += wd.distance.side[0] * gain;
      if (wd.wall[1] && std::abs(wd.diff.side[1]) < diff_thr)
        sc.position.y -= wd.distance.side[1] * gain;
    }
#endif
  }
  void wall_cut(const float velocity) {
#if SEARCH_WALL_CUT_ENABLED
    if (velocity < 120)
      return;
    for (int i = 0; i < 2; i++) {
      const float normal_th = sc.position.th / (M_PI / 2);
      const float frac_part = normal_th - roundf(normal_th);
      const float diff_thr = velocity * 2.0f;
      if (wd.diff.side[i] < -diff_thr && std::abs(frac_part) < 0.01f) {
        bz.play(Buzzer::SHORT);
        const float wall_cut_x = -18;
        auto th = roundf(sc.position.th / (M_PI / 2)) * M_PI / 2;
        auto offset_x = offset.rotate(-offset.th - th).x;
        auto fixed = sc.position.rotate(-th);
        auto x = offset_x + fixed.x;
        auto fixed_x = roundf((x - wall_cut_x) / 90) * 90 + wall_cut_x;
        fixed.x = fixed_x - offset_x;
        sc.position = fixed.rotate(th);
      }
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
    const float extra = distance - sc.position.x;
    if (extra > 0) {
      const float jerk = 500000;
      const float accel = 6000;
      const float v_start = sc.ref_v.tra;
      TrajectoryTracker tt(model::tt_gain);
      tt.reset(v_start);
      AccelDesigner ad(jerk, accel, v_start, v_max, v_end, extra);
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
        wall_avoid();
        wall_cut(ref.v);
        int_y += sc.position.y;
        // sc.position.th += int_y * 0.00000001f;
      }
      if (v_end < 1.0f)
        sc.set_target(0, 0);
    }
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
      wall_cut(ref.v);
    }
    sc.set_target(velocity, 0);
    const auto net = sd.get_net_curve();
    sc.position = (sc.position - net).rotate(-net.th);
    offset += net.rotate(offset.th);
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
  void task() override {
    const float velocity = SEARCH_RUN_VELOCITY;
    const float v_max = SEARCH_RUN_V_MAX;
    /* スタート */
    sc.enable();
    while (1) {
      if (q.empty())
        isRunningFlag = false;
      /* Actionがキューされるまで直進で待つ */
      float v = sc.ref_v.tra;
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (q.empty()) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        if (v > 0)
          v -= 3;
        xLastWakeTime = xTaskGetTickCount();
        Position cur = sc.position;
        float th = atan2f(-cur.y, (2 + 20 * velocity / 240)) - cur.th;
        sc.set_target(v, 40 * th);
      }
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
      switch (action) {
      case RobotBase::Action::START_STEP:
        sc.position.clear();
        imu.angle = 0;
        offset =
            Position(field::SegWidthFull / 2,
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
    vTaskDelay(portMAX_DELAY);
  }
};
