#pragma once

#include "config/model.h"
#include "global.h"

#include "Agent.h"
#include "Maze.h"
#include "RobotBase.h"
#include "TaskBase.h"
#include <Arduino.h>

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
    MazeLib::Position(7, 0), MazeLib::Position(8, 0), MazeLib::Position(7, 1), \
        MazeLib::Position(8, 1),                                               \
  }
#elif GOAL == 5
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
  void print() const { maze.print(); }
  bool isRunning() const { return isRunningFlag; }
  void set_goal(const std::vector<MazeLib::Position> &goal) {
    replaceGoals(goal);
  }
  bool backup() {
    {
      std::ifstream f(MAZE_BACKUP_PATH, std::ifstream::ate);
      const auto size = static_cast<size_t>(f.tellg());
      if (backupCounter < size / sizeof(WallLog))
        std::remove(MAZE_BACKUP_PATH);
    }
    std::ofstream f(MAZE_BACKUP_PATH, std::ios::binary | std::ios::app);
    if (f.fail()) {
      log_e("can't open file!: ", MAZE_BACKUP_PATH);
      bz.play(Buzzer::ERROR);
      return false;
    }
    const auto &wallLog = getMaze().getWallLogs();
    while (backupCounter < wallLog.size()) {
      const auto &wl = wallLog[backupCounter];
      f.write((const char *)&wl, sizeof(wl));
      backupCounter++;
    }
    return true;
  }
  bool restore() {
    std::ifstream f(MAZE_BACKUP_PATH, std::ios::binary);
    if (f.fail()) {
      log_e("can't open file!: ", MAZE_BACKUP_PATH);
      bz.play(Buzzer::ERROR);
      return false;
    }
    maze.reset();
    backupCounter = 0;
    while (!f.eof()) {
      WallLog wl;
      f.read((char *)(&wl), sizeof(WallLog));
      MazeLib::Position v = MazeLib::Position(wl.x, wl.y);
      maze.updateWall(v, wl.d, wl.b);
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
  int trace_count = 0;
  bool crashed_flag = false;

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
    if (!calcShortestDirections(sr.rp_fast.diag_enabled)) {
      printf("Couldn't solve the maze!\n");
      bz.play(Buzzer::ERROR);
      return false;
    }
    auto shortestDirs = getShortestDirections();
    shortestDirs.erase(shortestDirs.begin()); /*< 最初の直線を除去 */
    std::string path;
    Direction d = Direction::North;
    MazeLib::Position v(0, 1);
    for (const auto nextDir : shortestDirs) {
      v = v.next(nextDir);
      switch (Direction(nextDir - d)) {
      case Direction::Front:
        path += MazeLib::RobotBase::Action::ST_FULL;
        break;
      case Direction::Left:
        path += MazeLib::RobotBase::Action::TURN_L;
        break;
      case Direction::Right:
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
      setForceGoingToGoal();
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
      maze.resetLastWall(6);  //< クラッシュ後を想定して少し消す
      mt.drive(-0.2f, -0.2f); /*< 背中を確実に壁につける */
      delay(500);
      mt.free();
      setForceGoingToGoal();
      trace_count++; //< 0 -> 1
      for (int i = 0; i < trace_count; ++i)
        bz.play(Buzzer::SHORT7);
      if (!searchRun()) {
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      bz.play(Buzzer::COMPLETE);
      readyToStartWait();
      // sr.rp_fast.diag_enabled = false;
    }
    /* 最短 */
    while (1) {
      trace_count++; //< 1 -> 2
      if (trace_count == 6)
        break;
      if (!fastRun())
        waitForever();
      bz.play(Buzzer::SUCCESSFUL);
      readyToStartWait();
      /* 斜めを有効化 */
      sr.rp_fast.diag_enabled = true;
      /* 完走した場合はパラメータを上げる */
      if (sr.rp_fast.diag_enabled) {
        sr.rp_fast.curve_gain *= sr.rp_fast.cg_gain;
        sr.rp_fast.max_speed *= sr.rp_fast.ms_gain;
        sr.rp_fast.accel *= sr.rp_fast.ac_gain;
      }
      /* 最終走行だけ例外処理 */
      if (trace_count == 4 && !crashed_flag)
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
