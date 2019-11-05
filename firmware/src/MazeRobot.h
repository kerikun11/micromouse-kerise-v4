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

#define GOAL 1
#if GOAL == 1
#define MAZE_GOAL                                                              \
  { MazeLib::Position(1, 0) }
#elif GOAL == 2
#define MAZE_GOAL                                                              \
  {                                                                            \
    MazeLib::Position(6, 9), MazeLib::Position(6, 10),                         \
        MazeLib::Position(7, 9), MazeLib::Position(7, 10),                     \
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
    MazeLib::Position(19, 20), MazeLib::Position(19, 21),                      \
        MazeLib::Position(19, 22), MazeLib::Position(20, 20),                  \
        MazeLib::Position(20, 21), MazeLib::Position(20, 22),                  \
        MazeLib::Position(21, 20), MazeLib::Position(21, 21),                  \
        MazeLib::Position(21, 22),                                             \
  }
#endif
#define MAZE_SAVE_PATH "/spiffs/maze_backup.bin"
#define STATE_SAVE_PATH "/spiffs/maze_state.bin"

class MazeRobot : public RobotBase, private TaskBase {
public:
  struct State {
    int try_count = 0;             /**< 走行回数 */
    bool has_reached_goal = false; /**< ゴール区画にたどり着いたか */

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
    state.restore();
    for (int i = 0; i < state.try_count; i++)
      bz.play(Buzzer::SHORT7);
    for (int i = 0; i < state.try_count - 1; i++) {
      ma.rp_fast.curve_gain *= ma.rp_fast.cg_gain;
      ma.rp_fast.max_speed *= ma.rp_fast.ms_gain;
      ma.rp_fast.accel *= ma.rp_fast.ac_gain;
    }
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
    left = wd.wall[0];
    right = wd.wall[1];
    front = wd.wall[2];
    // bz.play(Buzzer::SHORT6);
#if 0
    /* 前1区画先の壁を読める場合 */
    if (!front)
      updateWall(current_pose.p.next(current_pose.d), current_pose.d,
                 tof.getDistance() < 210);
#endif
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
        // prevState == SearchAlgorithm::SEARCHING_FOR_GOAL &&
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

  bool fastRun() {
    if (!calcShortestDirections(ma.rp_fast.diag_enabled))
      return false;
    /* 最短経路の作成 */
    const auto path = convertDirectionsToSearch(getShortestDirections());
    ma.set_path(path);

    //> FastRun Start
    ma.enable();
    while (ma.isRunning())
      delay(100);
    ma.disable();
    //< FastRun End

    // ゴールで回収されるか待つ
    readyToStartWait();

    // 帰る
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
    /* 自己位置同定 */
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
      /* 最短走行でクラッシュした場合，パラメータを若干落とす */
      if (!state.has_reached_goal) {
        /* 最短中のクラッシュ */
        ma.rp_fast.curve_gain /= std::sqrt(ma.rp_fast.cg_gain);
        ma.rp_fast.max_speed /= std::sqrt(ma.rp_fast.ms_gain);
        ma.rp_fast.accel /= std::sqrt(ma.rp_fast.ac_gain);
      }
    }
    /* 探索 (強制探索モードまたは経路がひとつもない場合) */
    if (isForceSearch || !calcShortestDirections(true)) {
      maze.resetLastWalls(12); //< クラッシュ後を想定して少し消す
      mt.drive(-0.2f, -0.2f);  /*< 背中を確実に壁につける */
      delay(500);
      mt.free();
      state.newRun(); //< 0 -> 1
      if (!searchRun()) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      bz.play(Buzzer::COMPLETE);
      readyToStartWait();
    }
    /* 最短 */
    while (1) {
      state.newRun(); //< 1 -> 2
      if (!fastRun()) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      bz.play(Buzzer::SUCCESSFUL);
      readyToStartWait();
      /* 完走した場合はパラメータを上げる */
      ma.rp_fast.curve_gain *= ma.rp_fast.cg_gain;
      ma.rp_fast.max_speed *= ma.rp_fast.ms_gain;
      ma.rp_fast.accel *= ma.rp_fast.ac_gain;
    }
    /* 5走終了 */
    bz.play(Buzzer::COMPLETE);
    waitForever();
  }
};
