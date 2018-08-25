#pragma once

#include "Agent.h"
#include "Maze.h"
#include "TaskBase.h"
#include <Arduino.h>
#include <SPIFFS.h>

using namespace MazeLib;

/* Hardware */
#include "buzzer.h"
extern Buzzer bz;
#include "button.h"
extern Button btn;
#include "led.h"
extern LED led;
#include "motor.h"
extern Motor mt;
#include "fan.h"
extern Fan fan;
#include "imu.h"
extern IMU imu;
#include "encoder.h"
extern Encoder enc;
#include "reflector.h"
extern Reflector ref;
#include "tof.h"
extern ToF tof;

/* Software */
#include "UserInterface.h"
extern UserInterface ui;
#include "SpeedController.h"
extern SpeedController sc;
#include "WallDetector.h"
extern WallDetector wd;
#include "SearchRun.h"
extern SearchRun sr;
#include "FastRun.h"
extern FastRun fr;

#define MAZE_SOLVER_TASK_PRIORITY 2
#define MAZE_SOLVER_STACK_SIZE 8192

#define GOAL 1
#if GOAL == 0
#define MAZE_GOAL                                                              \
  { Vector(1, 0) }
#elif GOAL == 1
#define MAZE_GOAL                                                              \
  { Vector(9, 9), Vector(9, 10), Vector(10, 9), Vector(10, 10) }
#elif GOAL == 2
#define MAZE_GOAL                                                              \
  { Vector(3, 3), Vector(4, 4), Vector(4, 3), Vector(3, 4) }
#elif GOAL == 3
#define MAZE_GOAL                                                              \
  { Vector(7, 7), Vector(7, 8), Vector(8, 7), Vector(8, 8) }
#elif GOAL == 4
#define MAZE_GOAL                                                              \
  {                                                                            \
    Vector(19, 20), Vector(19, 21), Vector(19, 22), Vector(20, 20),            \
        Vector(20, 21), Vector(20, 22), Vector(21, 20), Vector(21, 21),        \
        Vector(21, 22),                                                        \
  }
#endif
#define MAZE_BACKUP_PATH "/maze_backup.bin"

class MazeSolver : TaskBase {
public:
  MazeSolver() : agent(MAZE_GOAL) {}
  void start(bool isForceSearch = false) {
    this->isForceSearch = isForceSearch;
    terminate();
    isRunningFlag = true;
    createTask("Maze Solver", MAZE_SOLVER_TASK_PRIORITY,
               MAZE_SOLVER_STACK_SIZE);
  }
  void terminate() {
    deleteTask();
    sr.disable();
    fr.disable();
    isRunningFlag = false;
  }
  void forceBackToStart() { agent.forceBackToStart(); }
  void print() {
    agent.printInfo();
    agent.printPath();
  }
  bool isRunning() { return isRunningFlag; }
  void set_goal(const std::vector<Vector> &goal) { agent.replaceGoals(goal); }
  bool isComplete() { return agent.isComplete(); }
  bool backup() {
    {
      File file = SPIFFS.open(MAZE_BACKUP_PATH, FILE_READ);
      if (backupCounter < file.size() / sizeof(WallLog)) {
        file.close();
        SPIFFS.remove(MAZE_BACKUP_PATH);
        bz.play(Buzzer::MAZE_BACKUP);
      }
    }
    // for (int i = 0; i < 400; i++)
    // maze.getWallLogs().push_back(WallLog(Vector(0, 0), Dir::North, false));
    // // for debug
    File file = SPIFFS.open(MAZE_BACKUP_PATH, FILE_APPEND);
    if (!file) {
      log_e("Can't open file!");
      bz.play(Buzzer::ERROR);
      return false;
    }
    const auto &wallLog = agent.getMaze().getWallLogs();
    while (backupCounter < wallLog.size()) {
      const auto &wl = wallLog[backupCounter];
      file.write((uint8_t *)&wl, sizeof(wl));
      backupCounter++;
    }
    bz.play(Buzzer::MAZE_BACKUP);
    return true;
  }
  bool restore() {
    File file = SPIFFS.open(MAZE_BACKUP_PATH, FILE_READ);
    if (!file) {
      log_e("Can't open file!");
      return false;
    }
    agent.getMaze().reset();
    backupCounter = 0;
    while (file.available()) {
      WallLog wl;
      file.read((uint8_t *)&wl, sizeof(WallLog));
      Vector v = Vector(wl.x, wl.y);
      Dir d = Dir(wl.d);
      bool b = wl.b;
      agent.getMaze().updateWall(v, d, b);
      backupCounter++;
    }
    return true;
  }

private:
  Agent agent;
  bool isForceSearch = false;
  bool isRunningFlag = false;
  int backupCounter = 0;

  void stopAndBackup() {
    sr.set_action(SearchRun::STOP);
    sr.waitForEnd();
    sr.disable();
    backup();
    imu.calibration(true);
    sr.set_action(SearchRun::RETURN);
    sr.set_action(SearchRun::GO_HALF);
    sr.enable();
  }
  void queueActions(const std::vector<Dir> &nextDirs) {
    int straight_count = 0;
    for (auto nextDir : nextDirs) {
      const auto &nextVec = agent.getCurVec().next(nextDir);
      switch (Dir(nextDir - agent.getCurDir())) {
      case Dir::Front:
        straight_count++;
        break;
      case Dir::Left:
        if (straight_count)
          sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
        straight_count = 0;
        sr.set_action(SearchRun::TURN_LEFT_90);
        break;
      case Dir::Back:
        if (straight_count)
          sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
        straight_count = 0;
        //            sr.set_action(SearchRun::TURN_BACK);
        stopAndBackup();
        break;
      case Dir::Right:
        if (straight_count)
          sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
        straight_count = 0;
        sr.set_action(SearchRun::TURN_RIGHT_90);
        break;
      }
      agent.updateCurVecDir(nextVec, nextDir);
    }
    if (straight_count)
      sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
    straight_count = 0;
  }
  bool searchRun(const bool isStartStep = true,
                 const Vector &startVec = Vector(0, 0),
                 const Dir &startDir = Dir::North) {
    if (!agent.isComplete())
      agent.getMaze().resetLastWall(5);
    agent.updateCurVecDir(startVec, startDir);
    auto res = agent.calcNextDirs();
    if (isStartStep) {
      if (res == SearchAlgorithm::Reached)
        return true;
      sr.set_action(SearchRun::START_STEP);
      agent.updateCurVecDir(startVec.next(startDir), startDir);
      agent.getMaze().resetLastWall(5);
    }
    // キャリブレーション
    bz.play(Buzzer::CONFIRM);
    imu.calibration();
    bz.play(Buzzer::CANCEL);
    sr.enable();
    while (1) {
      const auto &v = agent.getCurVec();
      const auto &d = agent.getCurDir();

      SearchAlgorithm::State prevState = agent.getState();
      uint32_t ms = millis();
      const auto status = agent.calcNextDirs(); //< 時間がかかる処理！
      printf("agent.calcNextDir(); %lu [ms]\n", millis() - ms);
      agent.printInfo(false);
      SearchAlgorithm::State newState = agent.getState();
      if (newState != prevState &&
          newState == SearchAlgorithm::SEARCHING_ADDITIONALLY) {
        bz.play(Buzzer::CONFIRM);
      }
      if (newState != prevState &&
          newState == SearchAlgorithm::BACKING_TO_START) {
        if (isStartStep)
          bz.play(Buzzer::COMPLETE);
      }

      // 既知区間移動をキューにつめる
      queueActions(agent.getNextDirs());

      // 探索終了
      if (status == SearchAlgorithm::Reached)
        break;
      //        if(status==SearchAlgorithm::Error) return false;

      sr.waitForEnd();

      // 壁を確認
      //        printf("ToF: %d, (passed: %d)\n", tof.getDistance(),
      //        tof.passedTimeMs());
      agent.updateWall(v, d, wd.wall[0], wd.wall[2], wd.wall[1], false);
      //        agent.updateWall(v, d + 1, wd.wall[0]); // left
      //        agent.updateWall(v, d + 0, wd.wall[2]); // front
      //        agent.updateWall(v, d - 1, wd.wall[1]); // right
      bz.play(Buzzer::SHORT);

      // 候補の中で行ける方向を探す
      Dir nextDir;
      if (!agent.findNextDir(v, d, nextDir)) {
        bz.play(Buzzer::ERROR);
        sr.set_action(SearchRun::STOP);
        sr.waitForEnd();
        sr.disable();
        waitForever();
        return false;
      }
      queueActions({nextDir});
    }
    sr.set_action(SearchRun::START_INIT);
    agent.updateCurVecDir(Vector(0, 0), Dir::North);
    agent.calcNextDirs(); //< 時間がかかる処理！
    sr.waitForEnd();
    sr.disable();
    backup();
    bz.play(Buzzer::COMPLETE);
    return true;
  }

  bool fast_run() {
    if (!agent.calcShortestDirs(fr.V90Enabled)) {
      printf("Couldn't solve the maze!\n");
      bz.play(Buzzer::ERROR);
      return false;
    }
    auto path = agent.getShortestDirs();
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

    // start drive
    fr.enable();
    fr.waitForEnd();
    fr.disable();
    // end drive

    readyToStartWait(); //< 回収されるか待つ

    //      agent.reset();
    sc.position.reset();
    sr.set_action(SearchRun::RETURN);
    sr.set_action(SearchRun::GO_HALF);
    return searchRun(false, v.next(d + 2), d + 2); //< 帰る
  }
  void readyToStartWait(const int wait_ms = 2000) {
    delay(200);
    for (int ms = 0; ms < wait_ms; ms++) {
      delay(1);
      if (fabs(imu.accel.z) > 9800 * 2) {
        bz.play(Buzzer::CANCEL);
        waitForever();
      }
    }
  }
  void waitForever() {
    delay(100);
    isRunningFlag = false;
    while (1)
      delay(1000);
  }
  virtual void task() {
    if (!agent.calcShortestDirs()) {
      if (!searchRun())
        waitForever();
    }
    fr.V90Enabled = false;
    if (!fast_run())
      waitForever();
    readyToStartWait();
    fr.V90Enabled = true;
    while (1) {
      if (!fast_run())
        waitForever();
      fr.runParameter.curve_gain *= 1.1f;
      fr.runParameter.max_speed *= 1.21f;
      fr.runParameter.accel *= 1.1f;
      fr.runParameter.decel *= 1.1f;
      readyToStartWait();
    }
  }
};
