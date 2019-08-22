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

#define GOAL 3
#if GOAL == 1
#define MAZE_GOAL                                                              \
  { Vector(1, 0) }
#elif GOAL == 2
#define MAZE_GOAL                                                              \
  { Vector(6, 9), Vector(6, 10), Vector(7, 9), Vector(7, 10) }
#elif GOAL == 3
#define MAZE_GOAL                                                              \
  { Vector(3, 3), Vector(4, 4), Vector(4, 3), Vector(3, 4) }
#elif GOAL == 4
#define MAZE_GOAL                                                              \
  { Vector(7, 0), Vector(8, 0), Vector(7, 1), Vector(8, 1) }
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
    maze.reset();
    backupCounter = 0;
    while (file.available()) {
      WallLog wl;
      file.read((uint8_t *)&wl, sizeof(WallLog));
      Vector v = Vector(wl.x, wl.y);
      Dir d = Dir(wl.d);
      bool b = wl.b;
      maze.updateWall(v, d, b);
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
    return sr.set_action(action);
  }
  void findWall(bool &left, bool &front, bool &right, bool &back) override {
    left = wd.wall[0];
    right = wd.wall[1];
    front = wd.wall[2];
    back = false;
    // bz.play(Buzzer::SHORT);
#if 0
    /* 前1区画先の壁を読める場合 */
    if (!front)
      updateWall(curVec.next(curDir), curDir, tof.getDistance() < 210);
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
    if (prevState == SearchAlgorithm::SEARCHING_FOR_GOAL) {
      bz.play(Buzzer::SUCCESSFUL);
    }
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
      bz.play(Buzzer::COMPLETE);
    }
    if (newState == SearchAlgorithm::REACHED_START) {
      bz.play(Buzzer::COMPLETE);
    }
  }
  void discrepancyWithKnownWall() override { bz.play(Buzzer::ERROR); }

  bool fastRun() {
    if (!calcShortestDirs(sr.rp_fast.diag_enabled)) {
      printf("Couldn't solve the maze!\n");
      bz.play(Buzzer::ERROR);
      return false;
    }
    auto shortestDirs = getShortestDirs();
    shortestDirs.erase(shortestDirs.begin()); /*< 最初の直線を除去 */
    std::string path;
    Dir d = Dir::North;
    Vector v(0, 1);
    for (auto nextDir : shortestDirs) {
      v = v.next(nextDir);
      switch (Dir(nextDir - d)) {
      case Dir::Front:
        path += MazeLib::RobotBase::Action::ST_FULL;
        break;
      case Dir::Left:
        path += MazeLib::RobotBase::Action::TURN_L;
        break;
      case Dir::Right:
        path += MazeLib::RobotBase::Action::TURN_R;
        break;
      default:
        return false; //< あってはならない
      }
      d = nextDir;
    }
    sr.set_path(path);
    // FastRun
    sr.enable();
    while (sr.isRunning()) {
      delay(100);
    }
    sr.disable();

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
    /* 自己位置同定 */
    if (isPositionIdentifying) {
      readyToStartWait();
      isPositionIdentifying = false;
      if (!sr.positionRecovery()) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      if (!positionIdentifyRun()) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      bz.play(Buzzer::COMPLETE);
    }
    /* 探索 */
    if (isForceSearch || !calcShortestDirs(true)) {
      maze.resetLastWall(6);  //< クラッシュ後を想定して少し消す
      mt.drive(-0.2f, -0.2f); /*< 背中を確実に壁につける */
      delay(500);
      mt.free();
      if (!searchRun()) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      bz.play(Buzzer::COMPLETE);
      readyToStartWait();
      // sr.diag_enabled = false;
    }
    /* 最短 */
    while (1) {
      if (!fastRun())
        waitForever();
      bz.play(Buzzer::COMPLETE);
      readyToStartWait();
      if (sr.rp_fast.diag_enabled) {
        sr.rp_fast.curve_gain *= sr.rp_fast.cg_gain;
        sr.rp_fast.max_speed *= sr.rp_fast.ms_gain;
        sr.rp_fast.accel *= sr.rp_fast.ac_gain;
      }
      sr.rp_fast.diag_enabled = true;
    }
  }
};
