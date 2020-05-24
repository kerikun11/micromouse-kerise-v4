#pragma once

#include "config/model.h"
#include "global.h"

#include "Agent.h"
#include "Maze.h"
#include "RobotBase.h"
#include "TaskBase.h"

using namespace MazeLib;

#define GOAL_SELECT 1
#if GOAL_SELECT == 1
#define MAZE_GOAL                                                              \
  { MazeLib::Position(1, 0) }
#elif GOAL_SELECT == 2
#define MAZE_GOAL                                                              \
  {                                                                            \
    MazeLib::Position(4, 4), MazeLib::Position(4, 5), MazeLib::Position(5, 4), \
        MazeLib::Position(5, 5),                                               \
  }
#elif GOAL_SELECT == 3
#define MAZE_GOAL                                                              \
  {                                                                            \
    MazeLib::Position(3, 3), MazeLib::Position(4, 4), MazeLib::Position(4, 3), \
        MazeLib::Position(3, 4),                                               \
  }
#elif GOAL_SELECT == 4
#define MAZE_GOAL                                                              \
  {                                                                            \
    MazeLib::Position(7, 7), MazeLib::Position(7, 8), MazeLib::Position(8, 7), \
        MazeLib::Position(8, 8),                                               \
  }
#elif GOAL_SELECT == 5
#define MAZE_GOAL                                                              \
  { MazeLib::Position(8, 8) }
#endif

class MazeRobot : public RobotBase {
public:
  static constexpr int MAZE_ROBOT_TASK_PRIORITY = 2;
  static constexpr int MAZE_ROBOT_STACK_SIZE = 8192;
  static constexpr auto MAZE_SAVE_PATH = "/spiffs/maze_backup.bin";
  static constexpr auto STATE_SAVE_PATH = "/spiffs/maze_state.bin";

public:
#define MAZEROBOT_TIMEOUT_SELECT 0
#if MAZEROBOT_TIMEOUT_SELECT == 0 /* 32 x 32 */
  static constexpr int competition_limit_time_s = 10 * 60;
  static constexpr int expected_fast_run_time_s = 60;
#elif MAZEROBOT_TIMEOUT_SELECT == 1 /* 16 x 16 */
  static constexpr int competition_limit_time_s = 5 * 60;
  static constexpr int expected_fast_run_time_s = 30;
#elif MAZEROBOT_TIMEOUT_SELECT == 2 /* 9 x 9 */
  static constexpr int competition_limit_time_s = 150;
  static constexpr int expected_fast_run_time_s = 30;
#endif
  struct State {
    int try_count = 0;             /**< 走行回数 */
    int max_parameter = -1;        /**< 成功パラメータ */
    int running_parameter = 0;     /**< 走行パラメータ */
    bool has_reached_goal = false; /**< ゴール区画にたどり着いたか */
    bool is_fast_run = false;      /**< 最短走行かどうか */
    bool offset_ms = 0;            /**< リセット後を想定 */

    int getTimeSecond() const { return (offset_ms + millis()) / 1000; }
    bool save(const std::string filepath = STATE_SAVE_PATH) {
      offset_ms = millis();
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
  void reset() {
    Agent::reset();
    maze.backupWallRecordsToFile(MAZE_SAVE_PATH, true);
    state = State();
    state.save();
  }
  bool backup() {
    state.save();
    return maze.backupWallRecordsToFile(MAZE_SAVE_PATH);
  }
  bool restore() {
    state.restore();
    return maze.restoreWallRecordsFromFile(MAZE_SAVE_PATH);
  }
  bool autoRun(const bool isForceSearch = false) {
    /* 迷路のチェック */
    auto_maze_check();
    // /* 自動復帰: 任意 -> ゴール -> スタート */
    // if (!auto_pi_run())
    //   return false;
    /* 探索走行: スタート -> ゴール -> スタート */
    if (isForceSearch || !calcShortestDirections(true))
      if (!auto_search_run())
        return false;
    /* 最短走行: スタート -> ゴール -> スタート */
    while (1) {
      state.newRun(); //< 1 -> 2
      /* 5走終了離脱 */
      // if (state.try_count > 5)
      // break;
      if (!auto_fast_run()) {
        bz.play(Buzzer::ERROR);
        mt.emergency_stop(); //< 復帰用
        return false;
      }
      bz.play(Buzzer::SUCCESSFUL);
      if (ui.waitForPickup())
        return true;
    }
    /* 5走終了 */
    bz.play(Buzzer::COMPLETE);
    return true;
  }
  void print() const {
    for (const auto &wl : maze.getWallRecords())
      std::cout << wl << std::endl;
    maze.print();
  }
  void setGoals(const Positions &goal) { replaceGoals(goal); }
  const State &getState() const { return state; }

private:
  Maze maze;
  State state;
  bool prevIsForceGoingToGoal = false;
  bool isForceSearch = false;

  /* override virtual functions */
protected:
  void waitForEndAction() override {
    // delay(300); // for debug
    ma.waitForEndAction();
    if (mt.is_emergency())
      setBreakFlag();
  }
  void queueAction(const RobotBase::SearchAction action) override {
    return ma.enqueue_action(action);
  }
  void senseWalls(bool &left, bool &front, bool &right) override {
    left = ma.is_wall[0];
    right = ma.is_wall[1];
    front = ma.is_wall[2];
  }
  void backupMazeToFlash() override { backup(); }
  void stopDequeue() override { ma.disable(); }
  void startDequeue() override { ma.enable(MoveAction::TaskActionSearchRun); }
  void calibration() override {
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    enc.clearOffset();
  }
  void calcNextDirectionsPreCallback() override {
    /* ゴール判定用フラグ */
    prevIsForceGoingToGoal = isForceGoingToGoal;
    /* ウルトラマンタイマー */
    const int time_limit_s = competition_limit_time_s -
                             (5 - state.try_count) * expected_fast_run_time_s;
    if (!isForceBackToStart && state.getTimeSecond() > time_limit_s) {
      setForceBackToStart();
      bz.play(Buzzer::TIMEOUT);
    }
  }
  void
  calcNextDirectionsPostCallback(SearchAlgorithm::State prevState,
                                 SearchAlgorithm::State newState) override {
    const auto d = !getNextDirections().empty() ? getNextDirections().back()
                                                : current_pose.d;
    /* 未知区間加速の設定 */
    /* 自己位置同定中は，直進優先ではないので，未知区間加速できない */
    /* ゴール区画内で未知区間加速してしまうのも防ぐ必要があった */
    ma.continue_straight_if_no_front_wall =
        newState != SearchAlgorithm::GOING_TO_GOAL &&
        newState != SearchAlgorithm::IDENTIFYING_POSITION &&
        !getNextDirectionCandidates().empty() &&
        getNextDirectionCandidates()[0] == d;
    /* ゴール判定 */
    if (prevIsForceGoingToGoal && !isForceGoingToGoal) {
      state.has_reached_goal = true;
      bz.play(Buzzer::CONFIRM);
    }
    /* 探索情報のお知らせ */
    if (newState == prevState)
      return;
    if (prevState == SearchAlgorithm::SEARCHING_FOR_GOAL)
      bz.play(Buzzer::SUCCESSFUL);
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION)
      bz.play(Buzzer::COMPLETE);
    if (prevState == SearchAlgorithm::SEARCHING_ADDITIONALLY &&
        newState != SearchAlgorithm::IMPOSSIBLE && !isForceBackToStart)
      bz.play(Buzzer::COMPLETE);
  }
  void discrepancyWithKnownWall() override { bz.play(Buzzer::ERROR); }

private:
  bool auto_maze_check() {
    if (!isComplete())
      bz.play(Buzzer::CANCEL);
    if (!isSolvable())
      bz.play(Buzzer::ERROR);
    while (!isSolvable()) {
      maze.resetLastWalls(12); //< 探索可能になるまで壁を消す
      if (getMaze().getWallRecords().empty())
        reset(), bz.play(Buzzer::ERROR);
    }
    return true;
  }
  bool auto_pi_run() {
    /* 自動復帰: 任意 -> ゴール -> スタート */
    while (1) {
      /* 既知区間斜めを無効化 */
      ma.rp_search.diag_enabled = false;
      /* 姿勢復帰 */
      ma.enable(MoveAction::TaskActionPositionRecovery);
      ma.waitForEndAction();
      ma.disable();
      /* ゴール区画の訪問を指定 */
      setForceGoingToGoal(!state.has_reached_goal);
      /* 同定 */
      if (positionIdentifyRun())
        break;
      /* エラー処理 */
      if (mt.is_emergency())
        ma.emergency_release();
      else
        bz.play(Buzzer::ERROR);
      if (ui.waitForPickup())
        return false;
    }
    /* スタート位置に戻ってきた */
    bz.play(Buzzer::COMPLETE);
    if (ui.waitForPickup())
      return false;
    return true;
  }
  bool auto_search_run() {
    /* 探索走行: スタート -> ゴール -> スタート */
    state.newRun(); //< 0 -> 1
    if (!searchRun()) {
      /* エラー処理 */
      if (mt.is_emergency())
        ma.emergency_release();
      else {
        /* 探索失敗 */
        bz.play(Buzzer::ERROR);
        return false;
        /* ToDo: 迷路を編集して探索を再開 */
        /* ToDo: 姿勢復帰をせずとも自己位置同定を開始できる． */
      }
      /* 自動復帰 */
      if (ui.waitForPickup())
        return false;
      auto_pi_run();
    }
    bz.play(Buzzer::COMPLETE);
    if (ui.waitForPickup())
      return false;
    return true;
  }
  bool auto_fast_run() {
#if 0
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
      ma.rp_search.diag_enabled = true;
    }
    /* 残り時間が足りない場合 */
    const int remaining_time_s =
        competition_limit_time_s - state.getTimeSecond();
    if (remaining_time_s < expected_fast_run_time_s * (5 - state.try_count)) {
      ma.rp_fast.down(state.running_parameter), state.running_parameter = 0;
      ma.rp_search.diag_enabled = false;
      ma.rp_fast.diag_enabled = false;
      bz.play(Buzzer::TIMEOUT);
    }
    /* 保存 */
    state.is_fast_run = true, state.save();
#else
    /* 走行パラメータ選択 */
    if (state.is_fast_run) {
      /* クラッシュ後の場合 */
      ma.rp_fast.down(1), state.running_parameter -= 1;
    } else {
      /* 初回 or 完走した場合 */
      if (ma.rp_fast.diag_enabled) //< 斜めあり -> パラメータを上げる
        ma.rp_fast.up(2), state.running_parameter += 2;
      else //< 斜めなし -> 斜めあり
        ma.rp_fast.diag_enabled = true;
      ma.rp_search.diag_enabled = true;
    }
    /* 保存 */
    state.is_fast_run = true, state.save();
#endif
    /* 最短経路の作成 */
    if (!calcShortestDirections(ma.rp_fast.diag_enabled))
      return false;
    const auto search_path = convertDirectionsToSearch(getShortestDirections());
    // logi << search_path.size() << std::endl;
    //> FastRun Start
    ma.set_fast_path(search_path);
    ma.enable(MoveAction::TaskActionFastRun);
    ma.waitForEndAction();
    ma.disable();
    //< FastRun End
    /* 最短成功 */
    state.is_fast_run = false;
    state.max_parameter = state.running_parameter;
    state.save();
    /* ゴールで回収されるか待つ */
    if (ui.waitForPickup())
      return true;
    /* 帰る */
    return endFastRunBackingToStartRun();
  }
};
