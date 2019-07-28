#pragma once

#include "config/model.h"
#include "global.h"

#include "AccelDesigner.h"
#include "TaskBase.h"
#include <cmath>
#include <queue>
#include <string>
#include <vector>

#define FAST_RUN_TASK_PRIORITY 3
#define FAST_RUN_STACK_SIZE 8192

#define FAST_RUN_WALL_CUT_ENABLED 0

class FastRun : TaskBase {
public:
  struct RunParameter {
    RunParameter(const float curve_gain = 1.0, const float max_speed = 600,
                 const float accel = 4800)
        : curve_gain(curve_gain), max_speed(max_speed), accel(accel) {}
    float curve_gain;
    float max_speed;
    float accel;
    const float cg_gain = 1.05f;
    const float ms_gain = 1.2f;
    const float ac_gain = 1.1f;
    const RunParameter &operator=(const RunParameter &obj) {
      curve_gain = obj.curve_gain;
      max_speed = obj.max_speed;
      accel = obj.accel;
      return *this;
    }
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
  FastRun() {}
  RunParameter runParameter;
  bool wallAvoidFlag = true;
  bool wallAvoid45Flag = true;
  bool wallCutFlag = true;
  bool V90Enabled = true;
  float fanDuty = 0.4f;

  void start() {
    createTask("FastRun", FAST_RUN_TASK_PRIORITY, FAST_RUN_STACK_SIZE);
  }
  void terminate() {
    deleteTask();
    sc.disable();
    path = "";
  }
  bool isRunning() { return path != ""; }
  void set_path(std::string path) { this->path = path; }
  void set_action(MazeLib::RobotBase::FastAction action) {
    path += (char)action;
  }
  void printPosition(const char *name) const {
    printf("%s\tRel:(%06.1f, %06.1f, %06.3f)\t", name, getRelativePosition().x,
           getRelativePosition().y, getRelativePosition().th);
    printf("Abs:(%06.1f, %06.1f, %06.3f)\n", sc.position.x, sc.position.y,
           sc.position.th);
  }
  Position getRelativePosition() const {
    return (sc.position - origin).rotate(-origin.th);
  }
  void updateOrigin(Position passed) { origin += passed.rotate(origin.th); }

private:
  Position origin;
  std::string path;
  int path_index;
  bool prev_wall[2];

  static auto round2(auto value, auto div) {
    return floor((value + div / 2) / div) * div;
  }
  static auto saturate(auto src, auto sat) {
    return std::max(std::min(src, sat), -sat);
  }
  bool isAlong() {
    return (int)(std::abs(origin.th) * 180.0f / PI + 1) % 90 < 2;
  }
  bool isDiag() {
    return (int)(std::abs(origin.th) * 180.0f / PI + 45 + 1) % 90 < 2;
  }

  void wallAvoid(float remain) {
    /* 一定速より小さかったら行わない */
    if (sc.est_v.tra < 100.0f)
      return;
    /* 曲線なら前半しか行わない */
    if (std::abs(sc.position.th - origin.th) > M_PI * 0.1f)
      return;
    uint8_t led_flags = 0;
    /* 90 [deg] の倍数 */
    if (wallAvoidFlag && isAlong()) {
      const float gain = 0.01f;
      const float wall_diff_thr = 10;
      if (wd.wall[0] && std::abs(wd.diff.side[0]) < wall_diff_thr) {
        sc.position +=
            Position(0, wd.distance.side[0] * gain, 0).rotate(origin.th);
        led_flags |= 8;
      }
      if (wd.wall[1] && std::abs(wd.diff.side[1]) < wall_diff_thr) {
        sc.position -=
            Position(0, wd.distance.side[1] * gain, 0).rotate(origin.th);
        led_flags |= 1;
      }
    }
    /* 45 [deg] の倍数 */
    if (wallAvoid45Flag && isDiag() && remain > field::SegWidthFull / 3) {
      const float shift = 0.04f;
      const float threashold = -50;
      if (wd.distance.front[0] > threashold) {
        sc.position += Position(0, shift, 0).rotate(origin.th);
        led_flags |= 4;
      }
      if (wd.distance.front[1] > threashold) {
        sc.position -= Position(0, shift, 0).rotate(origin.th);
        led_flags |= 2;
      }
    }
    led = led_flags;
  }
  void wallCut() {
    if (!wallCutFlag)
      return;
    if (!V90Enabled)
      return;
    /* 曲線なら前半しか使わない */
    if (std::abs(sc.position.th - origin.th) > M_PI * 0.1f)
      return;
#if FAST_RUN_WALL_CUT_ENABLED
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
  /* wall_dist = 90 - st_prev */
  void wall_calib(const float velocity, const float wall_dist) {
    /* 一定よりずれが大きければ処理を行わない */
    if (std::abs(tof.getDistance() - wall_dist) > 20)
      return;
    float dist =
        tof.getDistance() - (5 + tof.passedTimeMs()) / 1000.0f * velocity;
    float pos = wall_dist - dist;
    Position prev = sc.position;
    Position fix = sc.position.rotate(-origin.th);
    fix.x = round2(fix.x, field::SegWidthFull) + pos;
    fix = fix.rotate(origin.th);
    if (fabs(prev.rotate(-origin.th).x - fix.rotate(-origin.th).x) < 15.0f) {
      sc.position = fix;
      bz.play(Buzzer::SHORT);
    }
  }
  void straight_x(const float distance, const float v_max, const float v_end) {
    if (distance < 0) {
      updateOrigin(Position(distance, 0, 0));
      return;
    }
    const float jerk = 500000;
    const float accel = runParameter.accel;
    const float v_start = sc.ref_v.tra;
    TrajectoryTracker tt(model::tt_gain);
    tt.reset(v_start);
    AccelDesigner ad(jerk, accel, v_start, v_max, v_end, distance);
    float int_y = 0;
    for (int i = 0; i < 2; i++)
      prev_wall[i] = wd.wall[i];
    portTickType xLastWakeTime = xTaskGetTickCount();
    for (float t = 0; t < ad.t_end(); t += 0.001f) {
      auto est_q = getRelativePosition();
      auto ref_q = Position(ad.x(t), 0);
      auto ref_dq = Position(ad.v(t), 0);
      auto ref_ddq = Position(ad.a(t), 0);
      auto ref_dddq = Position(ad.j(t), 0);
      auto ref = tt.update(est_q, sc.est_v, sc.est_a, ref_q, ref_dq, ref_ddq,
                           ref_dddq);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      float extra = distance - est_q.x;
      wallAvoid(extra);
      wallCut();
      int_y += getRelativePosition().y;
      sc.position.th += int_y * 0.00000001f;
    }
    if (v_end < 1.0f)
      sc.set_target(0, 0);
    updateOrigin(Position(distance, 0, 0));
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
      auto est_q = getRelativePosition();
      auto ref = tt.update(est_q, sc.est_v, sc.est_a, s.q, s.dq, s.ddq, s.dddq);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      wallAvoid(0);
      wallCut();
    }
    sc.set_target(velocity, 0);
    updateOrigin(sd.get_net_curve());
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

private:
  void task() override {
    // 最短走行用にパターンを置換
    path = MazeLib::RobotBase::pathConvertSearchToFast(path, V90Enabled);
    const float v_max = runParameter.max_speed;
    // キャリブレーション
    delay(500);
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    // 壁に背中を確実につける
    mt.drive(-0.2f, -0.2f);
    delay(200);
    mt.free();
    // 走行開始
    fan.drive(fanDuty);
    delay(500);  //< ファンの回転数が一定なるのを待つ
    sc.enable(); //< 速度コントローラ始動
    /* 初期位置を設定 */
    sc.position =
        Position(field::SegWidthFull / 2,
                 field::WallThickness / 2 + model::TailLength, M_PI / 2);
    origin = sc.position;
    /* 最初の直線を追加 */
    float straight =
        field::SegWidthFull / 2 - model::TailLength - field::WallThickness / 2;
    for (int path_index = 0; path_index < path.length(); path_index++) {
      printf("FastRun: %c, st => %.1f\n", path[path_index], straight);
      const auto action =
          static_cast<const MazeLib::RobotBase::FastAction>(path[path_index]);
      fast_run_switch(action, straight, runParameter);
    }
    /* 最後の直線を消化 */
    if (straight > 1.0f) {
      straight_x(straight, v_max, 0);
      straight = 0;
    }
    sc.set_target(0, 0);
    fan.drive(0);
    delay(100);
    sc.disable();
    bz.play(Buzzer::COMPLETE);
    path = "";
    vTaskDelay(portMAX_DELAY);
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
