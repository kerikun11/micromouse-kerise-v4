#pragma once

#include "config/model.h"
#include "global.h"

#include "Agent.h"
#include "Maze.h"
#include "RobotBase.h"
#include "TaskBase.h"
#include <Arduino.h>
#include <SPIFFS.h>

using namespace MazeLib;

#define MAZE_ROBOT_TASK_PRIORITY 2
#define MAZE_ROBOT_STACK_SIZE 8192

#define GOAL 1
#if GOAL == 0
#elif GOAL == 1
#define MAZE_GOAL                                                              \
  { Vector(1, 0) }
#elif GOAL == 2
#define MAZE_GOAL                                                              \
  { Vector(15, 15) }
#elif GOAL == 3
#define MAZE_GOAL                                                              \
  { Vector(4, 4), Vector(5, 5), Vector(5, 4), Vector(4, 5) }
#elif GOAL == 4
#define MAZE_GOAL                                                              \
  { Vector(9, 9), Vector(10, 10), Vector(10, 9), Vector(9, 10) }
#elif GOAL == 5
#define MAZE_GOAL                                                              \
  {                                                                            \
    Vector(19, 20), Vector(19, 21), Vector(19, 22), Vector(20, 20),            \
        Vector(20, 21), Vector(20, 22), Vector(21, 20), Vector(21, 21),        \
        Vector(21, 22),                                                        \
  }
#endif
#define MAZE_BACKUP_PATH "/maze_backup.bin"

class MazeRobot : public RobotBase, private TaskBase {
public:
  MazeRobot() : RobotBase(maze) { replaceGoals(MAZE_GOAL); }

  void start(bool isForceSearch = false, bool isPositionIdentifying = false) {
    this->isForceSearch = isForceSearch;
    this->isPositionIdentifying = isPositionIdentifying;
    terminate();
    isRunningFlag = true;
    createTask("MazeRobot", MAZE_ROBOT_TASK_PRIORITY, MAZE_ROBOT_STACK_SIZE);
  }
  void terminate() {
    deleteTask();
    sr.disable();
    fr.terminate();
    isRunningFlag = false;
  }
  void print() {
    printInfo();
    printPath();
  }
  bool isRunning() { return isRunningFlag; }
  void set_goal(const std::vector<Vector> &goal) { replaceGoals(goal); }
  bool backup() {
    {
      File file = SPIFFS.open(MAZE_BACKUP_PATH, FILE_READ);
      if (backupCounter < file.size() / sizeof(WallLog)) {
        file.close();
        SPIFFS.remove(MAZE_BACKUP_PATH);
      }
    }
    File file = SPIFFS.open(MAZE_BACKUP_PATH, FILE_APPEND);
    if (!file) {
      log_e("Can't open file!");
      bz.play(Buzzer::ERROR);
      return false;
    }
    const auto &wallLog = getMaze().getWallLogs();
    while (backupCounter < wallLog.size()) {
      const auto &wl = wallLog[backupCounter];
      file.write((uint8_t *)&wl, sizeof(wl));
      backupCounter++;
    }
    // bz.play(Buzzer::MAZE_BACKUP);
    return true;
  }
  bool restore() {
    File file = SPIFFS.open(MAZE_BACKUP_PATH, FILE_READ);
    if (!file) {
      log_e("Can't open file!");
      return false;
    }
    getMaze().reset();
    backupCounter = 0;
    while (file.available()) {
      WallLog wl;
      file.read((uint8_t *)&wl, sizeof(WallLog));
      Vector v = Vector(wl.x, wl.y);
      Dir d = Dir(wl.d);
      bool b = wl.b;
      getMaze().updateWall(v, d, b);
      backupCounter++;
    }
    return true;
  }

private:
  Maze maze;
  bool isForceSearch = false;
  bool isRunningFlag = false;
  bool isPositionIdentifying = false;
  bool prevIsForceGoingToGoal = false;
  int backupCounter = 0;

protected:
  void waitForEndAction() override {
    // delay(1200); // for debug
    while (sr.isRunning()) {
      delay(1);
    }
  }
  void queueAction(const Action action) override {
    switch (action) {
    case RobotBase::START_STEP:
      return sr.set_action(SearchRun::START_STEP);
    case RobotBase::START_INIT:
      return sr.set_action(SearchRun::START_INIT);
    case RobotBase::STOP_HALF:
      return sr.set_action(SearchRun::STOP);
    case RobotBase::TURN_LEFT_90:
      return sr.set_action(SearchRun::TURN_LEFT_90);
    case RobotBase::TURN_RIGHT_90:
      return sr.set_action(SearchRun::TURN_RIGHT_90);
    case RobotBase::ROTATE_LEFT_90:
      return;
    case RobotBase::ROTATE_RIGHT_90:
      return;
    case RobotBase::ROTATE_180:
      return sr.set_action(SearchRun::RETURN);
    case RobotBase::STRAIGHT_FULL:
      return sr.set_action(SearchRun::GO_STRAIGHT);
    case RobotBase::STRAIGHT_HALF:
      return sr.set_action(SearchRun::GO_HALF);
    }
  }
  void findWall(bool &left, bool &front, bool &right, bool &back) override {
    left = wd.wall[0];
    right = wd.wall[1];
    front = wd.wall[2];
    back = false;
    bz.play(Buzzer::SHORT);
  }
  void backupMazeToFlash() override { backup(); }
  void stopDequeue() override {
    sr.disable();
    backup();
  }
  void startDequeue() override { sr.enable(); }
  void calibration() override {
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    enc.clearOffset();
  }
  void calcNextDirsPreCallback() override {
    prevIsForceGoingToGoal = isForceGoingToGoal;
  }
  void calcNextDirsPostCallback(SearchAlgorithm::State prevState,
                                SearchAlgorithm::State newState) override {
    if (!prevIsForceGoingToGoal && isForceGoingToGoal) {
      bz.play(Buzzer::CONFIRM);
    }
    if (newState == prevState)
      return;
    if (newState == SearchAlgorithm::SEARCHING_ADDITIONALLY) {
      bz.play(Buzzer::SUCCESSFUL);
    }
    if (newState == SearchAlgorithm::BACKING_TO_START) {
      bz.play(Buzzer::COMPLETE);
    }
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
      bz.play(Buzzer::COMPLETE);
    }
  }
  void discrepancyWithKnownWall() override { bz.play(Buzzer::ERROR); }

  bool fastRun() {
    if (!calcShortestDirs(fr.V90Enabled)) {
      printf("Couldn't solve the maze!\n");
      bz.play(Buzzer::ERROR);
      return false;
    }
    auto path = getShortestDirs();
    path.erase(path.begin());
    Dir d = Dir::North;
    Vector v(0, 1);
    for (auto nextDir : path) {
      v = v.next(nextDir);
      switch (Dir(nextDir - d)) {
      case Dir::Front:
        fr.set_action(FastRun::FAST_GO_STRAIGHT);
        break;
      case Dir::Left:
        fr.set_action(FastRun::FAST_TURN_LEFT_90);
        break;
      case Dir::Right:
        fr.set_action(FastRun::FAST_TURN_RIGHT_90);
        break;
      default:
        return false; //< あってはならない
      }
      d = nextDir;
    }
    // FastRun
    fr.start();
    while (fr.isRunning()) {
      delay(100);
    }
    fr.terminate();

    // 回収されるか待つ
    readyToStartWait();

    // 帰る
    return endFastRunBackingToStartRun();
  }
  void readyToStartWait(const int wait_ms = 2000) {
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
    // 自己位置同定
    if (isPositionIdentifying) {
      readyToStartWait();
      isPositionIdentifying = false;
      if (!sr.positionRecovery()) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      readyToStartWait();
      forceGoingToGoal();
      if (!positionIdentifyRun(Dir::East)) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      bz.play(Buzzer::COMPLETE);
    }
    // 探索
    if (isForceSearch || !calcShortestDirs(true)) {
      getMaze().resetLastWall(6); //< クラッシュ後を想定して少し消す
      forceGoingToGoal();
      mt.drive(-0.2f, -0.2f);
      delay(500);
      mt.free();
      if (!searchRun()) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      bz.play(Buzzer::COMPLETE);
      readyToStartWait();
      // fr.V90Enabled = false; //< for debug
    }
    // 最短
    while (1) {
      if (!fastRun())
        waitForever();
      bz.play(Buzzer::COMPLETE);
      readyToStartWait();
      fr.runParameter.curve_gain *= 1.1f;
      fr.runParameter.max_speed *= 1.1f;
      fr.runParameter.accel *= 1.1f;
      fr.runParameter.decel *= 1.1f;
      fr.V90Enabled = true;
    }
  }
};
