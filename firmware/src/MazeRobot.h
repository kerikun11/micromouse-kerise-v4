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
#define MAZE_BACKUP_PATH "/spiffs/maze_backup.bin"

class MazeRobot : public RobotBase, private TaskBase {
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
    sr.disable();
    isRunningFlag = false;
  }
  void reset() {
    Agent::reset();
    std::remove(MAZE_BACKUP_PATH);
  }
  void print() const { maze.print(); }
  bool isRunning() const { return isRunningFlag; }
  void set_goal(const Positions &goal) { replaceGoals(goal); }
  bool backup() { return maze.backupWallLogsFromFile(MAZE_BACKUP_PATH); }
  bool restore() { return maze.restoreWallLogsFromFile(MAZE_BACKUP_PATH); }

private:
  Maze maze;
  bool isForceSearch = false;
  bool isRunningFlag = false;
  bool isPositionIdentifying = false;
  bool prevIsForceGoingToGoal = false;
  int track_count = 0;
  bool crashed_flag = false;

protected:
  void waitForEndAction() override {
    // delay(1200); // for debug
    while (sr.isRunning())
      delay(1);
  }
  void queueAction(const RobotBase::Action action) override {
    return sr.set_action(action);
  }
  void senseWalls(bool &left, bool &front, bool &right) override {
    left = wd.wall[0];
    right = wd.wall[1];
    front = wd.wall[2];
    bz.play(Buzzer::SHORT6);
#if 0
    /* 前1区画先の壁を読める場合 */
    if (!front)
      updateWall(current_pose.p.next(current_pose.d), current_pose.d,
                 tof.getDistance() < 210);
#endif
  }
  void backupMazeToFlash() override { backup(); }
  void stopDequeue() override { sr.disable(); }
  void startDequeue() override { sr.enable(); }
  void calibration() override {
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    enc.clearOffset();
  }
  void calcNextDirectionsPreCallback() override {
    prevIsForceGoingToGoal = isForceGoingToGoal;
  }
  void
  calcNextDirectionsPostCallback(SearchAlgorithm::State prevState,
                                 SearchAlgorithm::State newState) override {
    if (!prevIsForceGoingToGoal && isForceGoingToGoal)
      bz.play(Buzzer::CONFIRM);
    if (newState == prevState)
      return;
    /* State Change has occurred */
    if (prevState == SearchAlgorithm::SEARCHING_FOR_GOAL)
      bz.play(Buzzer::SUCCESSFUL);
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION)
      bz.play(Buzzer::COMPLETE);
    if (prevState == SearchAlgorithm::SEARCHING_ADDITIONALLY)
      bz.play(Buzzer::COMPLETE);
  }
  void discrepancyWithKnownWall() override { bz.play(Buzzer::ERROR); }

  bool fastRun() {
    if (!calcShortestDirections(sr.rp_fast.diag_enabled))
      return false;
    /* 最短経路の作成 */
    const auto path = convertDirectionsToSearch(getShortestDirections());
    sr.set_path(path);

    //> FastRun Start
    sr.enable();
    while (sr.isRunning())
      delay(100);
    sr.disable();
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
      sr.positionRecovery();
      if (!positionIdentifyRun()) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      bz.play(Buzzer::COMPLETE);
      readyToStartWait();
      /* パラメータを若干落とす */
      sr.rp_fast.curve_gain /= std::sqrt(sr.rp_fast.cg_gain);
      sr.rp_fast.max_speed /= std::sqrt(sr.rp_fast.ms_gain);
      sr.rp_fast.accel /= std::sqrt(sr.rp_fast.ac_gain);
      crashed_flag = true;
    }
    /* 探索 (強制探索モードまたは経路がひとつもない場合) */
    if (isForceSearch || !calcShortestDirections(true)) {
      maze.resetLastWalls(6); //< クラッシュ後を想定して少し消す
      mt.drive(-0.2f, -0.2f); /*< 背中を確実に壁につける */
      delay(500);
      mt.free();
      track_count++; //< 0 -> 1
      if (!searchRun()) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      bz.play(Buzzer::COMPLETE);
      readyToStartWait();
    }
    /* 最短 */
    while (1) {
      track_count++; //< 1 -> 2
      // if (track_count== 6)
      //   break;
      if (!fastRun()) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      bz.play(Buzzer::SUCCESSFUL);
      readyToStartWait();
      /* 完走した場合はパラメータを上げる */
      sr.rp_fast.curve_gain *= sr.rp_fast.cg_gain;
      sr.rp_fast.max_speed *= sr.rp_fast.ms_gain;
      sr.rp_fast.accel *= sr.rp_fast.ac_gain;
      /* 最終走行だけ例外処理 */
      if (track_count == 4 && !crashed_flag)
        for (int i = 0; i < 1; i++) {
          sr.rp_fast.curve_gain *= sr.rp_fast.cg_gain;
          sr.rp_fast.max_speed *= sr.rp_fast.ms_gain;
          sr.rp_fast.accel *= sr.rp_fast.ac_gain;
          bz.play(Buzzer::CONFIRM);
        }
    }
    /* 5走終了 */
    bz.play(Buzzer::COMPLETE);
    waitForever();
  }
};
