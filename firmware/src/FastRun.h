#pragma once

#include "config/model.h"
#include "global.h"

#include "AccelDesigner.h"
#include "TaskBase.h"
#include <algorithm> //< std::replace
#include <cmath>
#include <queue>
#include <string>
#include <vector>

#define FAST_RUN_TASK_PRIORITY 3
#define FAST_RUN_STACK_SIZE 8192

#define FAST_RUN_WALL_CUT_ENABLED 0

class FastRun : TaskBase {
public:
  enum FAST_ACTION : char {
    FAST_GO_STRAIGHT = 's',
    FAST_GO_HALF = 'x',
    FAST_DIAGONAL_LEFT = 'w',
    FAST_DIAGONAL_RIGHT = 'W',
    FAST_TURN_LEFT_45 = 'z',
    FAST_TURN_RIGHT_45 = 'c',
    FAST_TURN_LEFT_45R = 'Z',
    FAST_TURN_RIGHT_45R = 'C',
    FAST_TURN_LEFT_90 = 'l',
    FAST_TURN_RIGHT_90 = 'r',
    FAST_TURN_LEFT_V90 = 'p',
    FAST_TURN_RIGHT_V90 = 'P',
    FAST_TURN_LEFT_S90 = 'q',
    FAST_TURN_RIGHT_S90 = 'Q',
    FAST_TURN_LEFT_135 = 'a',
    FAST_TURN_RIGHT_135 = 'd',
    FAST_TURN_LEFT_135R = 'A',
    FAST_TURN_RIGHT_135R = 'D',
    FAST_TURN_LEFT_180 = 'u',
    FAST_TURN_RIGHT_180 = 'U',
  };
  struct RunParameter {
    RunParameter(const float curve_gain = 1.0, const float max_speed = 600,
                 const float accel = 2400, const float decel = 2400)
        : curve_gain(curve_gain), max_speed(max_speed), accel(accel),
          decel(decel) {}
    RunParameter(std::array<float, 4> params)
        : curve_gain(params[0]), max_speed(params[1]), accel(params[2]),
          decel(params[3]) {}
    float curve_gain;
    float max_speed;
    float accel, decel;
    const RunParameter &operator=(const RunParameter &obj) {
      curve_gain = obj.curve_gain;
      max_speed = obj.max_speed;
      accel = obj.accel;
      decel = obj.decel;
      return *this;
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
  void set_action(FAST_ACTION action, const int num = 1) {
    for (int i = 0; i < num; i++)
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
  void setPosition(Position pos = Position(field::SegWidthFull / 2,
                                           field::WallThickness / 2 +
                                               model::TailLength,
                                           M_PI / 2)) {
    origin = pos;
    sc.position = pos;
  }

private:
  Position origin;
  std::string path;
  bool prev_wall[2];

  static auto round2(auto value, auto div) {
    return floor((value + div / 2) / div) * div;
  }
  static auto saturate(auto src, auto sat) {
    return std::max(std::min(src, sat), -sat);
  }

  void wallAvoid(bool diag, float remain) {
    // 一定速より小さかったら行わない
    if (sc.est_v.tra < 100.0f)
      return;
    uint8_t led_flags = 0;
    // 90 [deg] の倍数
    if (wallAvoidFlag && (int)(fabs(origin.th) * 180.0f / PI + 1) % 90 < 2) {
      const float gain = 0.008f;
      if (wd.wall[0]) {
        sc.position +=
            Position(0, wd.distance.side[0] * gain, 0).rotate(origin.th);
        led_flags |= 8;
      }
      if (wd.wall[1]) {
        sc.position -=
            Position(0, wd.distance.side[1] * gain, 0).rotate(origin.th);
        led_flags |= 1;
      }
    }
    // 45 [deg] の倍数
    // if (diag && wallAvoid45Flag && remain > SEGMENT_DIAGONAL_WIDTH / 3 &&
    //     (int)(fabs(origin.th) * 180.0f / PI + 45 + 1) % 90 < 2) {
    //   const float shift = 0.01f;
    //   const float threashold = -50;
    //   if (wd.distance.front[0] > threashold) {
    //     sc.position += Position(0, shift, 0).rotate(origin.th);
    //     led_flags |= 4;
    //   }
    //   if (wd.distance.front[1] > threashold) {
    //     sc.position -= Position(0, shift, 0).rotate(origin.th);
    //     led_flags |= 2;
    //   }
    // }
    led = led_flags;
  }
  void wallCut(bool diag) {
#if FAST_RUN_WALL_CUT_ENABLED
#define WALL_CUT_OFFSET_X_ (-30)
    if (wallCutFlag && fabs(origin.th - sc.position.th) < PI / 48) {
      for (int i = 0; i < 2; i++) {
        // 45 [deg] + 90 [deg] の倍数
        // if (diag && (int)(fabs(origin.th) * 180.0f / PI + 45 + 1) % 90 <
        // 4) {
        //   if (prev_wall[i] && !wd.wall[i]) {
        //     Position prev = sc.position;
        //     Position fix = sc.position.rotate(-origin.th);
        //     const float extra = 36;
        //     if (i == 0) {
        //       fix.x = round2(fix.x - extra - SEGMENT_DIAGONAL_WIDTH / 2,
        //                      SEGMENT_DIAGONAL_WIDTH) +
        //               SEGMENT_DIAGONAL_WIDTH / 2 + extra;
        //     } else {
        //       fix.x = round2(fix.x - extra, SEGMENT_DIAGONAL_WIDTH) + extra;
        //     }
        //     fix = fix.rotate(origin.th);
        //     if (fabs(prev.rotate(-origin.th).x -
        //              fix.rotate(-origin.th).x) < 10.0f) {
        //       sc.position = fix;
        //       printf("WallCutDiag[%d] X_ (%.1f, %.1f, %.1f) => (%.1f, %.1f, "
        //              "%.1f)\n",
        //              i, prev.x, prev.y, prev.th * 180.0f / PI,
        //              sc.position.x, sc.position.y, sc.position.th * 180 /
        //              PI);
        //     }
        //     bz.play(Buzzer::SHORT);
        //   }
        // }
        // 90 [deg] の倍数
        if ((int)(fabs(origin.th) * 180.0f / PI + 1) % 90 < 4) {
          if (prev_wall[i] && !wd.wall[i]) {
            Position prev = sc.position;
            Position fix = sc.position.rotate(-origin.th);
            fix.x = round2(fix.x, field::SegWidthFull) + WALL_CUT_OFFSET_X_;
            fix = fix.rotate(origin.th);
            if (fabs(prev.rotate(-origin.th).x - fix.rotate(-origin.th).x) <
                15.0f) {
              sc.position = fix;
              printf(
                  "WallCut[%d] X_ (%.1f, %.1f, %.1f) => (%.1f, %.1f, %.1f)\n",
                  i, prev.x, prev.y, prev.th * 180.0f / PI, sc.position.x,
                  sc.position.y, sc.position.th * 180 / PI);
              bz.play(Buzzer::SHORT);
            }
          }
        }
        prev_wall[i] = wd.wall[i];
      }
    }
#endif
  }
  void wall_calib(const float velocity) {
    if (wd.wall[2]) {
      float dist =
          tof.getDistance() - (5 + tof.passedTimeMs()) / 1000.0f * velocity;
      float pos = 90 - dist - SS_FLS90.straight_prev;
      Position prev = sc.position;
      Position fix = sc.position.rotate(-origin.th);
      fix.x = floor((fix.x + field::SegWidthFull / 2) / field::SegWidthFull) *
                  field::SegWidthFull +
              pos;
      fix = fix.rotate(origin.th);
      if (fabs(prev.rotate(-origin.th).x - fix.rotate(-origin.th).x) < 15.0f)
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
      wallAvoid(true, extra);
      wallCut(true);
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
      if (fabs(getRelativePosition().th) < 0.01f * PI) {
        wallAvoid(false, 0);
        wallCut(false);
      }
    }
    sc.set_target(velocity, 0);
    updateOrigin(sd.get_net_curve());
  }
  std::string replace(std::string &src, std::string from, std::string to) {
    if (from.empty())
      return src;
    auto pos = src.find(from);
    auto toLen = to.length();
    while ((pos = src.find(from, pos)) != std::string::npos) {
      src.replace(pos, from.length(), to);
      pos += toLen;
    }
    return src;
  }
  std::string pathConvertSearchToFast(std::string src, bool diag) {
    // スタートとゴールの半区画分を追加
    if (src[0] != 'x' && src[0] != 'c' && src[0] != 'z') {
      src = "x" + src + "x";
    }
    // 最短走行用にパターンを置換
    printf("Input Path: %s\n", src.c_str());
    if (diag) {
      replace(src, "s", "xx");
      replace(src, "l", "LL");
      replace(src, "r", "RR");

      replace(src, "RLLLLR", "RLpLR");
      replace(src, "LRRRRL", "LRPRL");

      replace(src, "xLLR", "zLR");
      replace(src, "xRRL", "cRL");
      replace(src, "LRRx", "LRC");
      replace(src, "RLLx", "RLZ");

      replace(src, "xLLLLR", "aLR");
      replace(src, "xRRRRL", "dRL");
      replace(src, "RLLLLx", "RLA");
      replace(src, "LRRRRx", "LRD");

      replace(src, "xLLLLx", "u");
      replace(src, "xRRRRx", "U");

      replace(src, "RLLR", "RLwLR");
      replace(src, "LRRL", "LRWRL");

      replace(src, "RL", "");
      replace(src, "LR", "");
      replace(src, "xRRx", "r");
      replace(src, "xLLx", "l");
    } else {
      replace(src, "s", "xx");

      replace(src, "xllx", "u");
      replace(src, "xrrx", "U");

      replace(src, "l", "q");
      replace(src, "r", "Q");
    }
    printf("Running Path: %s\n", src.c_str());
    return src;
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

public:
  void task() override {
    // 最短走行用にパターンを置換
    path = pathConvertSearchToFast(path, V90Enabled);
    const float v_max = runParameter.max_speed;
    // キャリブレーション
    delay(500);
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    // 壁に背中を確実につける
    mt.drive(-0.2f, -0.2f);
    delay(200);
    mt.free();
    imu.angle = 0;
    // 走行開始
    fan.drive(fanDuty);
    delay(500);  //< ファンの回転数が一定なるのを待つ
    sc.enable(); //< 速度コントローラ始動
    setPosition();
    float straight =
        field::SegWidthFull / 2 - model::TailLength - field::WallThickness / 2;
    for (int path_index = 0; path_index < path.length(); path_index++) {
      printf("FastRun: %c, st => %.1f\n", path[path_index], straight);
      switch (path[path_index]) {
      case FAST_TURN_LEFT_45:
        SlalomProcess(SS_FL45, straight, false, runParameter);
        break;
      case FAST_TURN_RIGHT_45:
        SlalomProcess(SS_FR45, straight, false, runParameter);
        break;
      case FAST_TURN_LEFT_45R:
        SlalomProcess(SS_FL45, straight, true, runParameter);
        break;
      case FAST_TURN_RIGHT_45R:
        SlalomProcess(SS_FR45, straight, true, runParameter);
        break;
      case FAST_TURN_LEFT_V90:
        SlalomProcess(SS_FLV90, straight, false, runParameter);
        break;
      case FAST_TURN_RIGHT_V90:
        SlalomProcess(SS_FRV90, straight, false, runParameter);
        break;
      case FAST_TURN_LEFT_S90:
        SlalomProcess(SS_FLS90, straight, false, runParameter);
        break;
      case FAST_TURN_RIGHT_S90:
        SlalomProcess(SS_FRS90, straight, false, runParameter);
        break;
      case FAST_TURN_LEFT_90:
        SlalomProcess(SS_FL90, straight, false, runParameter);
        break;
      case FAST_TURN_RIGHT_90:
        SlalomProcess(SS_FR90, straight, false, runParameter);
        break;
      case FAST_TURN_LEFT_135:
        SlalomProcess(SS_FL135, straight, false, runParameter);
        break;
      case FAST_TURN_RIGHT_135:
        SlalomProcess(SS_FR135, straight, false, runParameter);
        break;
      case FAST_TURN_LEFT_135R:
        SlalomProcess(SS_FL135, straight, true, runParameter);
        break;
      case FAST_TURN_RIGHT_135R:
        SlalomProcess(SS_FR135, straight, true, runParameter);
        break;
      case FAST_TURN_LEFT_180:
        SlalomProcess(SS_FL180, straight, false, runParameter);
        break;
      case FAST_TURN_RIGHT_180:
        SlalomProcess(SS_FR180, straight, false, runParameter);
        break;
      case FAST_GO_STRAIGHT:
        straight += field::SegWidthFull;
        break;
      case FAST_GO_HALF:
        straight += field::SegWidthFull / 2;
        break;
      case FAST_DIAGONAL_LEFT:
      case FAST_DIAGONAL_RIGHT:
        straight += field::SegWidthDiag / 2;
        break;
      }
    }
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
};
