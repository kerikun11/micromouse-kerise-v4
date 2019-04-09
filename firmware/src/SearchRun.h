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
#define SEARCH_WALL_CUT_ENABLED 1
#define SEARCH_WALL_FRONT_ENABLED 0
#define SEARCH_WALL_AVOID_ENABLED 1

#define SEARCH_RUN_TASK_PRIORITY 3
#define SEARCH_RUN_STACK_SIZE 8192

static constexpr const float ahead_length = 0.0f;

#define SEARCH_RUN_VELOCITY 180.0f
#define SEARCH_RUN_V_MAX 300.0f

class SearchRun : TaskBase {
public:
  enum ACTION {
    START_STEP,
    START_INIT,
    GO_STRAIGHT,
    GO_HALF,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    TURN_BACK,
    RETURN,
    STOP,
  };
  const char *action_string(enum ACTION action) {
    static const char name[][32] = {
        "start_step",    "start_init", "go_straight", "go_half", "turn_left_90",
        "turn_right_90", "turn_back",  "return",      "stop",
    };
    return name[action];
  }
  struct Operation {
    enum ACTION action;
    int num;
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
  void set_action(enum ACTION action, int num = 1) {
    struct Operation operation;
    operation.action = action;
    operation.num = num;
    q.push(operation);
    isRunningFlag = true;
  }
  bool isRunning() { return isRunningFlag; }
  //   int actions() const { return q.size(); }
  //   void waitForEnd() const { xSemaphoreTake(wait, portMAX_DELAY); }
  void printPosition(const char *name) const {
    printf("%s\tRel:(%.1f, %.1f, %.1f)\n", name, sc.position.x, sc.position.y,
           sc.position.th * 180 / PI);
  }
  bool positionRecovery() {
    sc.enable();
    for (int i = 0; i < 4; ++i) {
      if (wd.wall[2])
        wall_attach(true);
      turn(PI / 2);
    }
    while (1) {
      if (!wd.wall[2])
        break;
      wall_attach();
      turn(-PI / 2);
    }
    sc.disable();
    return true;
  }

private:
  std::queue<struct Operation> q;
  volatile bool isRunningFlag = false;
  Position offset;

  void wall_attach(bool force = false) {
#if SEARCH_WALL_ATTACH_ENABLED
    if ((force && tof.getDistance() < 180) || tof.getDistance() < 90 ||
        (wd.distance.front[0] > 10 && wd.distance.front[1] > 10)) {
      tof.disable();
      delay(10);
      portTickType xLastWakeTime = xTaskGetTickCount();
      WheelParameter wi;
      for (int i = 0; i < 3000; i++) {
        const float Kp = 60.0f;
        const float Ki = 90.0f;
        const float satu = 120.0f; //< [mm/s]
        const float end = 0.4f;
        WheelParameter wp;
        for (int j = 0; j < 2; ++j) {
          wp.wheel[j] = -wd.distance.front[j];
          wi.wheel[j] += wp.wheel[j] * 0.001f * Ki;
          wp.wheel[j] = wp.wheel[j] * Kp + wi.wheel[j];
          wp.wheel[j] = std::max(std::min(wp.wheel[j], satu), -satu);
        }
        if (std::pow(wp.wheel[0], 2) + std::pow(wp.wheel[1], 2) +
                std::pow(wi.wheel[0], 2) + std::pow(wi.wheel[1], 2) <
            end)
          break;
        wp.wheel2pole();
        sc.set_target(wp.tra, wp.rot);
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      }
      sc.set_target(0, 0);
      sc.position.x = 0;  //< 直進方向の補正
      sc.position.th = 0; //< 回転方向の補正
      tof.enable();
      bz.play(Buzzer::SHORT);
    }
#endif
  }
  void wall_avoid() {
#if SEARCH_WALL_AVOID_ENABLED
    if (std::abs(sc.position.th) < 0.05 * PI) {
      const float gain = 0.003;
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
    if (wd.wall[2]) {
      float value =
          tof.getDistance() - (5 + tof.passedTimeMs()) / 1000.0f * velocity;
      float x = sc.position.x;
      if (value > 60 && value < 120) {
        sc.position.x = 90 - value - ahead_length;
        bz.play(Buzzer::SELECT);
      }
      // sc.position.x = std::min(sc.position.x, 0.0f);
      // printf("FrontWallCalib: %.2f => %.2f\n", x, sc.position.x);
    }
#endif
  }
  void turn(const float angle, bool fix = false) {
    static constexpr float m_dddth = 4800 * M_PI;
    static constexpr float m_ddth = 48 * M_PI;
    static constexpr float m_dth = 4 * M_PI;
    imu.calibration();
    sc.enable();
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
      if (std::abs(Kp * error) + std::abs(Ki * int_error) < 0.01f * PI)
        break;
    }
    sc.set_target(0, 0);
    sc.position.th -= angle; //< 移動した量だけ位置を更新
    sc.position = sc.position.rotate(-angle); //< 移動した量だけ位置を更新
    offset += Position(0, 0, angle).rotate(offset.th);
    printPosition("Turn End");
  }
  void straight_x(const float distance, const float v_max, const float v_end) {
    const float jerk = 500000;
    const float accel = 3000;
    const float v_start = sc.ref_v.tra;
    TrajectoryTracker tt(model::tt_gain);
    tt.reset(v_start);
    AccelDesigner ad(jerk, accel, v_start, v_max, v_end, distance);
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
    for (int i = 0; i < max_v; i++) {
      sc.set_target(-i, -sc.position.th * 200.0f);
      delay(1);
    }
    for (int i = 0; i < 100; i++) {
      sc.set_target(-max_v, -sc.position.th * 200.0f);
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
    // bz.play(Buzzer::EMERGENCY);
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
  void task() override {
    const float velocity = SEARCH_RUN_VELOCITY;
    const float v_max = SEARCH_RUN_V_MAX;
    // スタート
    sc.enable();
    while (1) {
      //** SearchActionがキューされるまで直進で待つ
      if (q.empty())
        isRunningFlag = false;
      {
        float v = velocity;
        portTickType xLastWakeTime = xTaskGetTickCount();
        while (q.empty()) {
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
          xLastWakeTime = xTaskGetTickCount();
          Position cur = sc.position;
#define SEARCH_ST_LOOK_AHEAD(v) (5 + 20 * v / 240)
#define SEARCH_ST_FB_GAIN 40
          float th = atan2f(-cur.y, SEARCH_ST_LOOK_AHEAD(velocity)) - cur.th;
          sc.set_target(v, SEARCH_ST_FB_GAIN * th);
        }
      }
      struct Operation operation = q.front();
      q.pop();
      while (!q.empty()) {
        auto next = q.front();
        if (operation.action != next.action)
          break;
        operation.num += next.num;
        q.pop();
      }
      enum ACTION action = operation.action;
      int num = operation.num;
      printf("Action: %d %s\n", num, action_string(action));
      printPosition("Start");
      switch (action) {
      case START_STEP:
        sc.position.clear();
        imu.angle = 0;
        offset =
            Position(field::SegWidthFull / 2,
                     model::TailLength + field::WallThickness / 2, M_PI / 2);
        straight_x(field::SegWidthFull - model::TailLength -
                       field::WallThickness / 2 + ahead_length,
                   velocity, velocity);
        break;
      case START_INIT:
        straight_x(field::SegWidthFull / 2 - ahead_length, velocity, 0);
        wall_attach();
        turn(M_PI / 2);
        wall_attach();
        turn(M_PI / 2);
        put_back();
        mt.free();
        isRunningFlag = false;
        vTaskDelay(portMAX_DELAY);
      case GO_STRAIGHT:
        if (wd.wall[2])
          stop();
        straight_x(field::SegWidthFull * num, num > 1 ? v_max : velocity,
                   velocity);
        break;
      case GO_HALF:
        straight_x(field::SegWidthFull / 2 * num, velocity, velocity);
        break;
      case TURN_LEFT_90:
        for (int i = 0; i < num; i++) {
          if (wd.wall[0])
            stop();
          wall_calib(velocity);
          slalom::Trajectory st(SS_SL90);
          straight_x(st.get_straight_prev() - ahead_length, velocity, velocity);
          trace(st, velocity);
          straight_x(st.get_straight_post() + ahead_length, velocity, velocity);
        }
        break;
      case TURN_RIGHT_90:
        for (int i = 0; i < num; i++) {
          if (wd.wall[1])
            stop();
          wall_calib(velocity);
          slalom::Trajectory st(SS_SR90);
          straight_x(st.get_straight_prev() - ahead_length, velocity, velocity);
          trace(st, velocity);
          straight_x(st.get_straight_post() + ahead_length, velocity, velocity);
        }
        break;
      case TURN_BACK:
        straight_x(field::SegWidthFull / 2 - ahead_length, velocity, 0);
        uturn();
        straight_x(field::SegWidthFull / 2 + ahead_length, velocity, velocity);
        break;
      case RETURN:
        uturn();
        break;
      case STOP:
        straight_x(field::SegWidthFull / 2 - ahead_length, velocity, 0);
        // wall_attach();
        turn(0, true);
        sc.disable();
        isRunningFlag = false;
        vTaskDelay(portMAX_DELAY);
        break;
      }
      printPosition("End");
    }
    vTaskDelay(portMAX_DELAY);
  }
};
