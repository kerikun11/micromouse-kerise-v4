#pragma once

#include "config/model.h"
#include "global.h"

#include "Agent.h"
#include "Maze.h"
#include "RobotBase.h"
#include "TaskBase.h"

using namespace MazeLib;

#define MAZE_ROBOT_TASK_PRIORITY 2
#define MAZE_ROBOT_STACK_SIZE 8192

#define GOAL 4
#if GOAL == 1
#define MAZE_GOAL                                                              \
  { MazeLib::Position(1, 0) }
#elif GOAL == 2
#define MAZE_GOAL                                                              \
  {                                                                            \
    MazeLib::Position(17, 13), MazeLib::Position(18, 13),                      \
        MazeLib::Position(19, 13), MazeLib::Position(17, 14),                  \
        MazeLib::Position(18, 14), MazeLib::Position(19, 14),                  \
        MazeLib::Position(17, 15), MazeLib::Position(18, 15),                  \
        MazeLib::Position(19, 15),                                             \
  }
#elif GOAL == 3
#define MAZE_GOAL                                                              \
  {                                                                            \
    MazeLib::Position(3, 3), MazeLib::Position(4, 4), MazeLib::Position(4, 3), \
        MazeLib::Position(3, 4),                                               \
  }
#elif GOAL == 4
#define MAZE_GOAL                                                              \
  {                                                                            \
    MazeLib::Position(3, 3), MazeLib::Position(4, 3), MazeLib::Position(5, 3), \
        MazeLib::Position(3, 4), MazeLib::Position(4, 4),                      \
        MazeLib::Position(5, 4), MazeLib::Position(3, 5),                      \
        MazeLib::Position(4, 5), MazeLib::Position(5, 5),                      \
  }
#endif

#define MAZE_SAVE_PATH "/spiffs/maze_backup.bin"
#define STATE_SAVE_PATH "/spiffs/maze_state.bin"

class MazeRobot : public RobotBase, private TaskBase {
public:
  struct State {
    int try_count = 0;             /**< 走行回数 */
    int max_parameter = -1;        /**< 成功パラメータ */
    int running_parameter = 0;     /**< 走行パラメータ */
    bool has_reached_goal = false; /**< ゴール区画にたどり着いたか */
    bool is_fast_run = false;      /**< 最短走行かどうか */

    bool save(const std::string filepath = STATE_SAVE_PATH) {
      std::ofstream of(filepath, std::ios::binary | std::ios::app);
      if (of.fail()) {
        loge << "failed to open file! " << filepath << std::endl;
        return false;
      }
      of.write((const char *)this, sizeof(State));
      return true;
    }
    bool restore(const std::string filepath = STATE_SAVE_PATH) const {
      std::ifstream f(filepath, std::ios::binary);
      if (f.fail()) {
        loge << "failed to open file! " << filepath << std::endl;
        return false;
      }
      f.read((char *)this, sizeof(State));
      return true;
    }
    void newRun() {
      try_count++;
      has_reached_goal = false;
      save();
    }
  };

public:
  MazeRobot() : RobotBase(maze) { replaceGoals(MAZE_GOAL); }

  void start(bool isForceSearch = false, bool isPositionIdentifying = false) {
    terminate();
    this->isForceSearch = isForceSearch;
    this->isPositionIdentifying = isPositionIdentifying;
    isRunningFlag = true;
    createTask("MazeRobot", MAZE_ROBOT_TASK_PRIORITY, MAZE_ROBOT_STACK_SIZE);
  }
  void terminate() {
    deleteTask();
    ma.disable();
    isRunningFlag = false;
  }
  void reset() {
    Agent::reset();
    maze.backupWallLogsToFile(MAZE_SAVE_PATH, true);
    state = State();
    state.save();
  }
  bool backup() {
    state.save();
    return maze.backupWallLogsToFile(MAZE_SAVE_PATH);
  }
  bool restore() {
    /* パラメータを復元する処理！！！ */
    // state.restore();
    state.try_count = 1; //< 探索済みの最短初回
    return maze.restoreWallLogsFromFile(MAZE_SAVE_PATH);
  }
  void autoRun(bool isForceSearch, bool isPositionIdentifying) {
    start(isForceSearch, isPositionIdentifying);
    while (isRunning()) {
      if (mt.isEmergency()) {
        bz.play(Buzzer::EMERGENCY);
        terminate();
        fan.free();
        delay(1000);
        mt.emergencyRelease();
        /* EmergencyStopのタイミング次第でdisabledの場合がある */
        tof.enable();
        start(false, true); /*< Position Identification Run */
      }
      delay(100);
    }
    terminate();
  }
  void print() const {
    for (const auto &wl : maze.getWallLogs()) {
      std::cout << wl << std::endl;
    }
    maze.print();
  }
  bool isRunning() const { return isRunningFlag; }
  void setGoals(const Positions &goal) { replaceGoals(goal); }
  const State &getState() const { return state; }

private:
  Maze maze;
  bool isForceSearch = false;
  bool isRunningFlag = false;
  bool isPositionIdentifying = false;
  bool prevIsForceGoingToGoal = false;
  State state;
  // int time_stamp_us = 0;

protected:
  void waitForEndAction() override {
    // delay(1200); // for debug
    while (ma.isRunning())
      delay(1);
  }
  void queueAction(const RobotBase::Action action) override {
    return ma.set_action(action);
  }
  void senseWalls(bool &left, bool &front, bool &right) override {
    left = ma.is_wall[0];
    right = ma.is_wall[1];
    front = ma.is_wall[2];
  }
  void backupMazeToFlash() override { backup(); }
  void stopDequeue() override { ma.disable(); }
  void startDequeue() override { ma.enable(); }
  void calibration() override {
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    enc.clearOffset();
  }
  void calcNextDirectionsPreCallback() override {
    prevIsForceGoingToGoal = isForceGoingToGoal;
    // time_stamp_us = esp_timer_get_time();
  }
  void
  calcNextDirectionsPostCallback(SearchAlgorithm::State prevState,
                                 SearchAlgorithm::State newState) override {
    const auto d = !getNextDirections().empty() ? getNextDirections().back()
                                                : current_pose.d;
    /* 未知区間加速の設定 */
    ma.continue_straight_if_no_front_wall =
        prevState == SearchAlgorithm::SEARCHING_FOR_GOAL &&
        newState != SearchAlgorithm::IDENTIFYING_POSITION &&
        !getNextDirectionCandidates().empty() &&
        getNextDirectionCandidates()[0] == d;
    /* ゴール判定 */
    if (prevIsForceGoingToGoal && !isForceGoingToGoal) {
      state.has_reached_goal = true;
      bz.play(Buzzer::CONFIRM);
    }
    if (isForceSearch)
      setForceBackToStart(true);
    // const auto now_us = esp_timer_get_time();
    // const float dur_us = now_us - time_stamp_us;
    // lgr.push({dur_us});
    /* State Change has occurred */
    if (newState == prevState)
      return;
    if (prevState == SearchAlgorithm::SEARCHING_FOR_GOAL)
      bz.play(Buzzer::SUCCESSFUL);
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION)
      bz.play(Buzzer::COMPLETE);
    if (prevState == SearchAlgorithm::SEARCHING_ADDITIONALLY)
      bz.play(Buzzer::COMPLETE);
  }
  void discrepancyWithKnownWall() override { bz.play(Buzzer::ERROR); }

  bool searchRun() {
    mt.drive(-0.2f, -0.2f); /*< 背中を確実に壁につける */
    delay(500);
    mt.free();
    return RobotBase::searchRun();
  }
  bool fastRun() {
    /* 走行パラメータ選択 */
    if (state.is_fast_run) {
      /* クラッシュ後の場合 */
      if (state.max_parameter < 0) { /*< 最短未成功の状態 */
        ma.rp_fast.diag_enabled = !ma.rp_fast.diag_enabled; //< 斜めを交互に
        ma.rp_fast.down(state.running_parameter), state.running_parameter = 0;
      }
    } else {
      /* 初回 or 完走した場合 */
      if (state.try_count == 2) //< 最短初回だけ特別にパラメータを上げる
        ma.rp_fast.up(2), state.running_parameter += 2;
      if (ma.rp_fast.diag_enabled) //< 斜めあり -> パラメータを上げる
        ma.rp_fast.up(2), state.running_parameter += 2;
      else //< 斜めなし -> 斜めあり
        ma.rp_fast.diag_enabled = true;
    }
    state.is_fast_run = true, state.save();
    /* 最短経路の作成 */
    if (!calcShortestDirections(ma.rp_fast.diag_enabled))
      return false;
    const auto path = convertDirectionsToSearch(getShortestDirections());
    ma.set_path(path);
    //> FastRun Start
    ma.enable();
    while (ma.isRunning())
      delay(100);
    ma.disable();
    //< FastRun End
    /* 最短成功 */
    state.is_fast_run = false;
    state.max_parameter = state.running_parameter;
    state.save();
    /* ゴールで回収されるか待つ */
    readyToStartWait();
    /* 帰る */
    return endFastRunBackingToStartRun();
  }
  void readyToStartWait(const int wait_ms = 1000) {
    led = 0xf;
    delay(200);
    for (int ms = 0; ms < wait_ms; ms++) {
      delay(1);
      if (fabs(imu.gyro.y) > PI) {
        bz.play(Buzzer::CANCEL);
        waitForever();
      }
    }
    led = 0x0;
  }
  void waitForever() {
    delay(100);
    isRunningFlag = false;
    vTaskDelay(portMAX_DELAY);
  }
  void task() override {
    /* 迷路のチェック */
    if (!calcShortestDirections(true)) {
      maze.resetLastWalls(12); //< クラッシュ後を想定して少し消す
    }
    /* 自動復帰: 任意 -> ゴール -> スタート */
    if (isPositionIdentifying) {
      isPositionIdentifying = false;
      readyToStartWait();
      ma.positionRecovery();
      if (!positionIdentifyRun(!state.has_reached_goal)) {
        bz.play(Buzzer::ERROR);
        mt.emergencyStop();
        waitForever();
      }
      bz.play(Buzzer::COMPLETE);
      readyToStartWait();
    }
    /* 探索走行: スタート -> ゴール -> スタート */
    if (isForceSearch || !calcShortestDirections(true)) {
      state.newRun(); //< 0 -> 1
      if (!searchRun()) {
        bz.play(Buzzer::ERROR);
        waitForever(); //< 復帰しない
      }
      bz.play(Buzzer::COMPLETE);
      readyToStartWait();
    }
    /* 最短走行: スタート -> ゴール -> スタート */
    while (1) {
      state.newRun(); //< 1 -> 2
      /* 5走終了 */
      if (state.try_count > 5)
        bz.play(Buzzer::COMPLETE);
      // break;
      if (!fastRun()) {
        bz.play(Buzzer::ERROR);
        mt.emergencyStop();
        waitForever();
      }
      bz.play(Buzzer::SUCCESSFUL);
      readyToStartWait();
    }
    /* 5走終了 */
    bz.play(Buzzer::COMPLETE);
    waitForever();
  }
};
