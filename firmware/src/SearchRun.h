#pragma once

#include "config/config.h"
#include "global.h"

#include "TaskBase.h"
#include <AccelDesigner.h>
#include <cmath>
#include <queue>
#include <vector>

#define SEARCH_WALL_ATTACH_ENABLED 1
#define SEARCH_WALL_CUT_ENABLED 0
#define SEARCH_WALL_FRONT_ENABLED 1
#define SEARCH_WALL_AVOID_ENABLED 1

#define SEARCH_END_REMAIN 5
#define SEARCH_ST_LOOK_AHEAD(v) (5 + 20 * v / 240)
#define SEARCH_ST_FB_GAIN 20
#define SEARCH_CURVE_FB_GAIN 4.0f

#define ahead_length 2

#define SEARCH_RUN_TASK_PRIORITY 3
#define SEARCH_RUN_STACK_SIZE 8192

#define SEARCH_RUN_VELOCITY 240.0f
#define SEARCH_RUN_V_CURVE 240.0f
#define SEARCH_RUN_V_MAX 600.0f

class SearchTrajectory {
public:
  SearchTrajectory() { reset(); }
  virtual ~SearchTrajectory() {}
  void reset() { last_index = -SEARCH_END_REMAIN; }
  Position getNextDir(const Position &cur, const float velocity) {
    int index_cur = getNextIndex(cur);
    Position dir = (getPosition(index_cur + 3) - cur).rotate(-cur.theta);
    float dt = 1.0f / velocity;
    float ff =
        (getPosition(last_index + 1).theta - getPosition(last_index).theta) /
        dt;
    dir.theta = ff + SEARCH_CURVE_FB_GAIN * atan2f(dir.y, dir.x);
    return dir;
  }
  float getRemain() const { return (getSize() - last_index) * interval; }
  Position getEndPosition() const { return getPosition(getSize()); }

protected:
  int last_index;
  const float interval = 1.0f;
  virtual int size() const { return 1; }
  virtual Position position(const int index) const {
    return Position(index * interval, 0, 0);
  }
  int getSize() const { return size(); }
  Position getPosition(const int index) const { return position(index); }
  int getNextIndex(const Position &pos) {
    for (int i = last_index;; i++) {
      Position target = getPosition(i);
      Position dir = (target - pos).rotate(-target.theta);
      if (dir.x > 0) {
        last_index = i;
        return last_index;
      }
    }
    return last_index;
  }
};

class S90 : public SearchTrajectory {
public:
  S90(bool mirror = false) : mirror(mirror) {}
  const float velocity = SEARCH_RUN_V_CURVE;
  const float straight = 1.0f;

private:
  bool mirror;
  virtual int size() const { return 71; }
  virtual Position position(int index) const {
    static const float data[71 + 1][3] = {
        {0.0000000000, 0.0000000000, 0.0000000000},
        {0.9999999901, 0.0000915712, 0.0003652106},
        {1.9999988262, 0.0014393753, 0.0028446510},
        {2.9999812693, 0.0070744275, 0.0091818750},
        {3.9998725787, 0.0214539765, 0.0204440724},
        {4.9994629773, 0.0496777091, 0.0368349648},
        {5.9983439280, 0.0965883197, 0.0576590881},
        {6.9959143616, 0.1659132508, 0.0814437245},
        {7.9914894784, 0.2596081526, 0.1062591208},
        {8.9844294170, 0.3780099662, 0.1311074535},
        {9.9741209539, 0.5210456014, 0.1559557863},
        {10.9599530354, 0.6886267474, 0.1808041190},
        {11.9413171888, 0.8806498822, 0.2056524518},
        {12.9176075226, 1.0969964756, 0.2305007845},
        {13.8882212298, 1.3375330053, 0.2553491173},
        {14.8525588869, 1.6021109571, 0.2801974500},
        {15.8100250219, 1.8905669645, 0.3050457828},
        {16.7600287081, 2.2027228892, 0.3298941155},
        {17.7019835637, 2.5383859637, 0.3547424483},
        {18.6353078372, 2.8973490227, 0.3795907810},
        {19.5594253090, 3.2793905030, 0.4044391138},
        {20.4737652018, 3.6842744702, 0.4292874465},
        {21.3777630737, 4.1117509623, 0.4541357793},
        {22.2708610157, 4.5615559331, 0.4789841120},
        {23.1525076520, 5.0334116986, 0.5038324448},
        {24.0221584823, 5.5270270496, 0.5286807776},
        {24.8792765573, 6.0420972524, 0.5535291103},
        {25.7233325411, 6.5783042163, 0.5783774431},
        {26.5538054730, 7.1353168680, 0.6032257758},
        {27.3701827870, 7.7127911894, 0.6280741086},
        {28.1719603299, 8.3103707509, 0.6529224413},
        {28.9586430618, 8.9276867323, 0.6777707741},
        {29.7297451213, 9.5643579298, 0.7026191068},
        {30.4847904304, 10.2199912524, 0.7274674396},
        {31.2233130051, 10.8941817643, 0.7523157723},
        {31.9448569558, 11.5865132010, 0.7771641051},
        {32.6489766372, 12.2965582813, 0.8020124378},
        {33.3352373362, 13.0238787068, 0.8268607706},
        {34.0032152025, 13.7680252857, 0.8517091033},
        {34.6524979203, 14.5285385942, 0.8765574361},
        {35.2826847722, 15.3049488967, 0.9014057688},
        {35.8933866392, 16.0967769345, 0.9262541016},
        {36.4842263963, 16.9035340186, 0.9511024343},
        {37.0548391948, 17.7247220299, 0.9759507671},
        {37.6048726871, 18.5598338688, 1.0007990998},
        {38.1339874042, 19.4083538442, 1.0256474326},
        {38.6418567555, 20.2697579732, 1.0504957654},
        {39.1281670734, 21.1435146100, 1.0753440981},
        {39.5926181134, 22.0290844475, 1.1001924309},
        {40.0349230206, 22.9259205757, 1.1250407636},
        {40.4548087502, 23.8334693031, 1.1498890964},
        {40.8520161668, 24.7511700759, 1.1747374291},
        {41.2263000445, 25.6784563573, 1.1995857619},
        {41.5774292138, 26.6147558666, 1.2244340946},
        {41.9051868862, 27.5594905785, 1.2492824274},
        {42.2093706515, 28.5120770329, 1.2741307601},
        {42.4897927631, 29.4719270816, 1.2989790929},
        {42.7462801463, 30.4384479183, 1.3238274256},
        {42.9786744024, 31.4110429843, 1.3486757584},
        {43.1868320272, 32.3891120048, 1.3735240911},
        {43.3706244860, 33.3720509955, 1.3983724239},
        {43.5299383122, 34.3592529306, 1.4232207566},
        {43.6646751777, 35.3501083657, 1.4480690894},
        {43.7747518930, 36.3440055843, 1.4729174221},
        {43.8601349541, 37.3403282624, 1.4975887283},
        {43.9215834395, 38.3384163875, 1.5205782448},
        {43.9617034515, 39.3375954195, 1.5400372863},
        {43.9846853187, 40.3373222759, 1.5547324671},
        {43.9955596346, 41.3372593492, 1.5642656230},
        {43.9993074321, 42.3372513328, 1.5691436188},
        {43.9999823853, 43.3372510052, 1.5706894826},
        {44.0000000000, 44.0000000000, 1.5707963268},
    };
    Position ret;
    if (index < 0) {
      ret = Position(0 + interval * index, 0, 0);
    } else if (index > size() - 1) {
      Position end(data[size()][0], data[size()][1], data[size()][2]);
      ret = end + Position((index - size()) * interval * cos(end.theta),
                           (index - size()) * interval * sin(end.theta), 0);
    } else {
      ret = Position(data[index][0], data[index][1], data[index][2]);
    }
    if (mirror)
      return ret.mirror_x();
    return ret;
  }
};

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
    printf("SearchRun Enabled\n");
    deleteTask();
    createTask("SearchRun", SEARCH_RUN_TASK_PRIORITY, SEARCH_RUN_STACK_SIZE);
  }
  void disable() {
    deleteTask();
    sc.disable();
    while (q.size()) {
      q.pop();
    }
    printf("SearchRun Disabled\n");
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
           sc.position.theta * 180 / PI);
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
  bool prev_wall[2];

  void wall_attach(bool force = false) {
#if SEARCH_WALL_ATTACH_ENABLED
    if ((force && tof.getDistance() < 180) || tof.getDistance() < 90 ||
        (wd.distance.front[0] > 10 && wd.distance.front[1] > 10)) {
      tof.disable();
      portTickType xLastWakeTime = xTaskGetTickCount();
      SpeedController::WheelParameter wi;
      for (int i = 0; i < 3000; i++) {
        const float Kp = 72.0f;
        const float Ki = 6.0f;
        const float satu = 200.0f; //< [mm/s]
        const float end = 0.4f;
        SpeedController::WheelParameter wp;
        for (int j = 0; j < 2; ++j) {
          wp.wheel[j] = -wd.distance.front[j];
          wi.wheel[j] += wp.wheel[j] * 0.001f * Ki;
          wp.wheel[j] = wp.wheel[j] * Kp + wi.wheel[j] * Ki;
          wp.wheel[j] = std::max(std::min(wp.wheel[j], satu), -satu);
        }
        wp.wheel2pole();
        if (std::abs(wp.wheel[0]) + std::abs(wp.wheel[1]) < end)
          break;
        sc.set_target(wp.tra, wp.rot);
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      }
      sc.set_target(0, 0);
      sc.position.x = 0;     //< 直進方向の補正
      sc.position.theta = 0; //< 回転方向の補正
      tof.enable();
      bz.play(Buzzer::SHORT);
    }
#endif
  }
  void wall_avoid(const float distance) {
#if SEARCH_WALL_AVOID_ENABLED
    if (std::abs(sc.position.theta) < 0.05 * PI) {
      const float gain = 0.0016f;
      if (wd.wall[0])
        sc.position.y += wd.distance.side[0] * gain;
      if (wd.wall[1])
        sc.position.y -= wd.distance.side[1] * gain;
    }
#endif
#if SEARCH_WALL_CUT_ENABLED
#define SEARCH_WALL_CUT_OFFSET_X_ 66
    for (int i = 0; i < 2; i++) {
      if (prev_wall[i] && !wd.wall[i] && sc.position.x > 30.0f) {
        const float prev_x = sc.position.x;
        float fix = -((int)sc.position.x) % SEGMENT_WIDTH +
                    SEARCH_WALL_CUT_OFFSET_X_ - ahead_length;
        if (distance > SEGMENT_WIDTH - 1 && fix < 0.0f) {
          sc.position.x = sc.position.x + fix;
          printf("WallCut[%d] X_ distance: %.0f, x: %.1f => %.1f\n", i,
                 distance, prev_x, sc.position.x);
          bz.play(Buzzer::CANCEL);
        }
      }
      // if (!prev_wall[i] && wd.wall[i] && sc.position.x > 30.0f) {
      //   const float prev_x = sc.position.x;
      //   if (distance > SEGMENT_WIDTH - 1)
      //     sc.position.x = sc.position.x - ((int)sc.position.x) %
      //     SEGMENT_WIDTH +
      //                     SEARCH_WALL_CUT_OFFSET__X - ahead_length;
      //   printf("WallCut[%d] _X distance: %.0f, x: %.1f => %.1f\n", i,
      //   distance,
      //          prev_x, sc.position.x);
      //   bz.play(Buzzer::CONFIRM);
      // }
      prev_wall[i] = wd.wall[i];
    }
#endif
  }
  void wall_calib(const float velocity) {
#if SEARCH_WALL_FRONT_ENABLED
    if (wd.wall[2]) {
      float value =
          tof.getDistance() - (5 + tof.passedTimeMs()) / 1000.0f * velocity;
      float x = sc.position.x;
      if (value > 60 && value < 120)
        sc.position.x = 90 - value - ahead_length + SEARCH_END_REMAIN;
      sc.position.x = std::min(sc.position.x, 0.0f);
      printf("FrontWallCalib: %.2f => %.2f\n", x, sc.position.x);
    }
#endif
  }
  void turn(const float angle, bool fix = false) {
    const float speed = 3 * M_PI;
    const float accel = 24 * M_PI;
    const float decel = 24 * M_PI;
    const float back_gain = 1.0f;
    int ms = 0;
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
      if (fix)
        break;
      if (std::abs(sc.est_v.rot) > speed)
        break;
      float delta = sc.position.x * cos(-sc.position.theta) -
                    sc.position.y * sin(-sc.position.theta);
      if (angle > 0) {
        sc.set_target(-delta * back_gain, ms / 1000.0f * accel);
      } else {
        sc.set_target(-delta * back_gain, -ms / 1000.0f * accel);
      }
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      xLastWakeTime = xTaskGetTickCount();
      ms++;
    }
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      xLastWakeTime = xTaskGetTickCount();
      float extra = angle - sc.position.theta;
      if (std::abs(sc.est_v.rot) < 0.1 && std::abs(extra) < 0.1)
        break;
      float target_speed = sqrt(2 * decel * std::abs(extra));
      float delta = sc.position.x * cos(-sc.position.theta) -
                    sc.position.y * sin(-sc.position.theta);
      target_speed = (target_speed > speed) ? speed : target_speed;
      if (extra > 0) {
        sc.set_target(-delta * back_gain, target_speed);
      } else {
        sc.set_target(-delta * back_gain, -target_speed);
      }
    }
    sc.set_target(0, 0);
    sc.position.theta -= angle; //< 移動した量だけ位置を更新
    sc.position = sc.position.rotate(-angle); //< 移動した量だけ位置を更新
    printPosition("Turn End");
  }
  void straight_x(const float distance, const float v_max, const float v_end) {
    const float accel = 6000;
    const float v_start = sc.ref_v.tra;
    AccelDesigner ad(accel, v_start, v_max, v_end,
                     distance - SEARCH_END_REMAIN);
    float int_y = 0;
    for (int i = 0; i < 2; i++)
      prev_wall[i] = wd.wall[i];
    portTickType xLastWakeTime = xTaskGetTickCount();
    for (float t = 0; true; t += 0.001f) {
      Position cur = sc.position;
      if (v_end >= 1.0f && cur.x > distance - SEARCH_END_REMAIN)
        break;
      if (v_end < 1.0f && cur.x > distance)
        break;
      float velocity = ad.v(t);
      if (v_end < 1.0f)
        velocity = std::max(60.0f, velocity);
      float theta = atan2f(-cur.y, SEARCH_ST_LOOK_AHEAD(velocity)) - cur.theta;
      if (velocity < 100)
        theta = 0;
      sc.set_target(velocity, SEARCH_ST_FB_GAIN * theta, ad.a(t));
      wall_avoid(distance);
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      int_y += sc.position.y;
      sc.position.theta += int_y * 0.00000001f;
    }
    if (v_end < 1.0f)
      sc.set_target(0, 0);
    sc.position.x -= distance; //< 移動した量だけ位置を更新
  }
  template <class C> void trace(C tr) {
    const float velocity = sc.ref_v.tra;
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
      if (tr.getRemain() < SEARCH_END_REMAIN)
        break;
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      xLastWakeTime = xTaskGetTickCount();
      Position dir = tr.getNextDir(sc.position, velocity);
      sc.set_target(velocity, dir.theta);
    }
    sc.set_target(velocity, 0);
    const Position end = tr.getEndPosition();
    sc.position = (sc.position - end).rotate(-end.theta);
    printPosition("Trace End");
  }
  void put_back() {
    const int max_v = 150;
    for (int i = 0; i < max_v; i++) {
      sc.set_target(-i, -sc.position.theta * 200.0f);
      delay(1);
    }
    for (int i = 0; i < 100; i++) {
      sc.set_target(-max_v, -sc.position.theta * 200.0f);
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
    bz.play(Buzzer::EMERGENCY);
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
          float theta =
              atan2f(-cur.y, SEARCH_ST_LOOK_AHEAD(velocity)) - cur.theta;
          sc.set_target(v, SEARCH_ST_FB_GAIN * theta);
          wall_avoid(0);
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
        straight_x(SEGMENT_WIDTH - MACHINE_TAIL_LENGTH - WALL_THICKNESS / 2 +
                       ahead_length,
                   velocity, velocity);
        break;
      case START_INIT:
        straight_x(SEGMENT_WIDTH / 2 - ahead_length, velocity, 0);
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
        straight_x(SEGMENT_WIDTH * num, num > 1 ? v_max : velocity, velocity);
        break;
      case GO_HALF:
        straight_x(SEGMENT_WIDTH / 2 * num, velocity, velocity);
        break;
      case TURN_LEFT_90:
        for (int i = 0; i < num; i++) {
          if (wd.wall[0])
            stop();
          S90 tr(false);
          wall_calib(velocity);
          straight_x(tr.straight - ahead_length, velocity, tr.velocity);
          trace(tr);
          straight_x(tr.straight + ahead_length, velocity, velocity);
        }
        break;
      case TURN_RIGHT_90:
        for (int i = 0; i < num; i++) {
          if (wd.wall[1])
            stop();
          S90 tr(true);
          wall_calib(velocity);
          straight_x(tr.straight - ahead_length, velocity, tr.velocity);
          trace(tr);
          straight_x(tr.straight + ahead_length, velocity, velocity);
        }
        break;
      case TURN_BACK:
        straight_x(SEGMENT_WIDTH / 2 - ahead_length, velocity, 0);
        uturn();
        straight_x(SEGMENT_WIDTH / 2 + ahead_length, velocity, velocity);
        break;
      case RETURN:
        uturn();
        break;
      case STOP:
        straight_x(SEGMENT_WIDTH / 2 - ahead_length, velocity, 0);
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
