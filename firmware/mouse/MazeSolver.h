#pragma once

#include <Arduino.h>
#include <SPIFFS.h>
#include "TaskBase.h"
#include "config.h"
#include "MazeLib/Maze.h"
#include "MazeLib/SearchAlgorithm.h"

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
#include "ExternalController.h"
extern ExternalController ec;
#include "Logger.h"
extern Logger lg;
#include "SearchRun.h"
extern SearchRun sr;
#include "FastRun.h"
extern FastRun fr;

//#define printf  lg.printf

#define MAZE_SOLVER_TASK_PRIORITY 2
#define MAZE_SOLVER_STACK_SIZE    8192

#define MAZE_GOAL           {Vector(1,0)}
//#define MAZE_GOAL           {Vector(3, 3), Vector(4, 4), Vector(4, 3), Vector(3, 4)}
//#define MAZE_GOAL           {Vector(7,7), Vector(7,8), Vector(8,7), Vector(8,8)}
//#define MAZE_GOAL           {Vector(19, 20), Vector(19, 21), Vector(19, 22), Vector(20, 20), Vector(20, 21), Vector(20, 22), Vector(21, 20), Vector(21, 21), Vector(21, 22)}
#define MAZE_BACKUP_SIZE    5

#define MAZE_BACKUP_PATH    "/maze_backup.bin"

class MazeSolver: TaskBase {
  public:
    MazeSolver(): searchAlgorithm(maze, MAZE_GOAL) {
      maze_backup.push_back(maze);
    }
    void start(bool isForceSearch = false) {
      this->isForceSearch = isForceSearch;
      terminate();
      isRunningFlag = true;
      createTask("Maze Solver", MAZE_SOLVER_TASK_PRIORITY, MAZE_SOLVER_STACK_SIZE);
    }
    void terminate() {
      deleteTask();
      sr.disable();
      fr.disable();
      isRunningFlag = false;
    }
    void forceBackToStart() {
      if (searchAlgorithm.getState() != SearchAlgorithm::SEARCHING_FOR_GOAL) {
        searchAlgorithm.forceBackToStart();
      }
    }
    void print() {
      //      int i = 0;
      //      for (auto& maze : maze_backup) {
      //        printf("Backup Maze %d: \n", i++);
      //        maze.print();
      //      }
      searchAlgorithm.printInfo();
      searchAlgorithm.printPath();
    }
    bool isRunning() {
      return isRunningFlag;
    }
    void set_goal(const std::vector<Vector>& goal) {
      searchAlgorithm.reset(goal);
    }
    bool backup() {
      uint32_t us = micros();
      File file = SPIFFS.open(MAZE_BACKUP_PATH, FILE_WRITE);
      if (!file) {
        log_e("Can't open file!");
        return false;
      }
      for (auto& maze : maze_backup) {
        file.write((const uint8_t*)(&maze), sizeof(Maze));
      }
      log_d("Backup: %d [us]", micros() - us);
      return true;
    }
    bool restore() {
      File file = SPIFFS.open(MAZE_BACKUP_PATH, FILE_READ);
      if (!file) {
        log_e("Can't open file!");
        return false;
      }
      while (file.available() >= sizeof(Maze)) {
        uint8_t data[sizeof(Maze)];
        file.read(data, sizeof(Maze));
        Maze m;
        memcpy((uint8_t*)(&m), data, sizeof(Maze));
        maze_backup.push_back(m);
        if (maze_backup.size() > MAZE_BACKUP_SIZE) maze_backup.pop_front();
      }
      maze = maze_backup.back();
      searchAlgorithm.reset();
      return true;
    }
  private:
    Maze maze;
    std::deque<Maze> maze_backup;
    SearchAlgorithm searchAlgorithm;
    bool isForceSearch = false;
    bool isRunningFlag = false;

    void queueActions(const std::vector<Dir>& nextDirs) {
      int straight_count = 0;
      for (auto nextDir : nextDirs) {
        Vector nextVec = searchAlgorithm.getCurVec().next(nextDir);
        switch (Dir(nextDir - searchAlgorithm.getCurDir())) {
          case Dir::Forward:
            straight_count++;
            break;
          case Dir::Left:
            if (straight_count) sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
            straight_count = 0;
            sr.set_action(SearchRun::TURN_LEFT_90);
            break;
          case Dir::Back:
            if (straight_count) sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
            straight_count = 0;
            sr.set_action(SearchRun::TURN_BACK);
            break;
          case Dir::Right:
            if (straight_count) sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
            straight_count = 0;
            sr.set_action(SearchRun::TURN_RIGHT_90);
            break;
        }
        searchAlgorithm.updateCurVecDir(nextVec, nextDir);
      }
      if (straight_count) sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
      straight_count = 0;
    }
    void stopAndBackup() {
      sr.set_action(SearchRun::STOP);
      sr.waitForEnd();
      sr.disable();
      backup();
      bz.play(Buzzer::MAZE_BACKUP);
      const auto& v = searchAlgorithm.getCurVec();
      const auto& d = searchAlgorithm.getCurDir();
      searchAlgorithm.updateCurVecDir(v.next(d + 2), d + 2); // u-turn
      sr.set_action(SearchRun::RETURN);
      sr.set_action(SearchRun::GO_HALF);
      sr.enable();
    }

    bool searchRun(const bool isStartStep = true, const Vector& startVec = Vector(0, 0), const Dir& startDir = Dir::North) {
      searchAlgorithm.reset();
      searchAlgorithm.updateCurVecDir(startVec, startDir);
      searchAlgorithm.calcNextDir();
      if (searchAlgorithm.getState() == SearchAlgorithm::REACHED_START) return true;
      if (isStartStep) {
        // queue Action::START_STEP
        sr.set_action(SearchRun::START_STEP);
        searchAlgorithm.updateCurVecDir(startVec.next(startDir), startDir);
      }
      // キューの消化を開始する
      sr.enable();
      while (1) {
        const auto& v = searchAlgorithm.getCurVec();
        const auto& d = searchAlgorithm.getCurDir();
        SearchAlgorithm::State prevState = searchAlgorithm.getState();
        uint32_t ms = millis();
        searchAlgorithm.calcNextDir(); //< 時間がかかる処理
        printf("searchAlgorithm.calcNextDir(); %lu [ms]\n", millis() - ms);
        printf("Cur: ( %3d, %3d, %3d), State: %s       \n", v.x, v.y, uint8_t(d), searchAlgorithm.stateString(searchAlgorithm.getState()));
        printf("nextDirs: ");
        for (auto d : searchAlgorithm.getNextDirs()) printf("%c", ">^<v"[d]);
        printf("    \n");
        printf("nextDirsInAdvance: ");
        for (auto d : searchAlgorithm.getNextDirsInAdvance()) printf("%c", ">^<v"[d]);
        printf("    \n");
        SearchAlgorithm::State newState = searchAlgorithm.getState();

        if (newState != prevState && newState == SearchAlgorithm::REACHED_GOAL) {
          /* REACHED_GOAL */
          bz.play(Buzzer::CONFIRM);
        }
        if (newState != prevState && newState == SearchAlgorithm::SEARCHING_ADDITIONALLY) {
          /* SEARCHING_ADDITIONALLY */
          bz.play(Buzzer::CONFIRM);
          stopAndBackup();
          continue;
        }
        if (newState != prevState && newState == SearchAlgorithm::BACKING_TO_START) {
          /* BACKING_TO_START */
          bz.play(Buzzer::COMPLETE);
          stopAndBackup();
          continue;
        }
        if (newState != prevState && newState == SearchAlgorithm::GOT_LOST) {
          /* GOT_LOST */
          bz.play(Buzzer::ERROR);
          sr.set_action(SearchRun::STOP);
          sr.waitForEnd();
          sr.disable();
          waitForever();
        }

        // 既知区間移動をキューにつめる
        const auto& nextDirs = searchAlgorithm.getNextDirs();
        queueActions(searchAlgorithm.getNextDirs());

        // 探索終了
        if (searchAlgorithm.getState() == SearchAlgorithm::REACHED_START) break;

        // ここでキューが消化されるまで待つ
        sr.waitForEnd();

        // 壁を確認
        //        printf("ToF: %d, (passed: %d)\n", tof.getDistance(), tof.passedTimeMs());
        //        if (maze.isKnown(v, d + 1) && maze.isWall(v, d + 1) != wd.wall[0]) bz.play(Buzzer::CANCEL);
        //        if (maze.isKnown(v, d + 0) && maze.isWall(v, d + 0) != wd.wall[2]) bz.play(Buzzer::CANCEL);
        //        if (maze.isKnown(v, d - 1) && maze.isWall(v, d - 1) != wd.wall[1]) bz.play(Buzzer::CANCEL);
        searchAlgorithm.updateWall(v, d + 1, wd.wall[0]); // left
        searchAlgorithm.updateWall(v, d + 0, wd.wall[2]); // front
        searchAlgorithm.updateWall(v, d - 1, wd.wall[1]); // right
        bz.play(Buzzer::SHORT);
        // 候補の中で行ける方向を探す
        const auto nextDirsInAdvance = searchAlgorithm.getNextDirsInAdvance();
        const auto nextDirInAdvance = *std::find_if(nextDirsInAdvance.begin(), nextDirsInAdvance.end(), [&](const Dir & dir) {
          return !maze.isWall(v, dir);
        });
        queueActions({nextDirInAdvance});

        // backup the maze
        maze_backup.push_back(maze);
        if (maze_backup.size() > MAZE_BACKUP_SIZE) maze_backup.pop_front();
      }
      if (searchAlgorithm.getState() != SearchAlgorithm::REACHED_START) return false;
      // queue Action::START_INIT
      sr.set_action(SearchRun::START_INIT);
      searchAlgorithm.updateCurVecDir(Vector(0, 0), Dir::North);
      // move robot here
      sr.waitForEnd();
      // stop robot here
      sr.disable();
      // 最短経路が導出できるか確かめる
      if (!searchAlgorithm.calcShortestDirs()) {
        printf("Couldn't solve the maze!\n");
        bz.play(Buzzer::ERROR);
        return false;
      }
      bz.play(Buzzer::COMPLETE);
      return true;
    }

    void fast_run() {
      auto path = searchAlgorithm.getShortestDirs();
      path.erase(path.begin());
      Dir d = Dir::North;
      Vector v(0, 1);
      for (auto nextDir : path) {
        v = v.next(nextDir);
        switch (Dir(nextDir - d)) {
          case Dir::East:
            fr.set_action(FastRun::FAST_GO_STRAIGHT);
            break;
          case Dir::North:
            fr.set_action(FastRun::FAST_TURN_LEFT_90);
            break;
          case Dir::West:
            break;
          case Dir::South:
            fr.set_action(FastRun::FAST_TURN_RIGHT_90);
            break;
        }
        d = nextDir;
      }

      // start drive
      fr.enable();
      fr.waitForEnd();
      fr.disable();
      // end drive
      readyToStartWait();

      searchAlgorithm.reset();
      if (searchAlgorithm.getState() != SearchAlgorithm::REACHED_START) {
        // 帰りの重ね探索
        bz.play(Buzzer::CONFIRM);
        printf("Additionally Searching\n");
        sc.position.reset();
        sr.set_action(SearchRun::RETURN);
        sr.set_action(SearchRun::GO_HALF);
        if (!searchRun(false, v.next(d + 2), d + 2)) while (1) delay(1000);
      } else {
        // back to start
        printf("Back to Start\n");
        sc.position.reset();
        sr.set_action(SearchRun::RETURN);
        sr.set_action(SearchRun::GO_HALF);
        path = searchAlgorithm.getShortestDirs();
        d = path.back();
        path.pop_back();
        std::reverse(path.begin(), path.end());
        int straight_count = 0;
        for (auto nextDir : path) {
          switch (Dir(nextDir - d)) {
            case Dir::East:
              straight_count++;
              break;
            case Dir::North:
              if (straight_count) sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
              straight_count = 0;
              sr.set_action(SearchRun::TURN_LEFT_90);
              break;
            case Dir::West:
              break;
            case Dir::South:
              if (straight_count) sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
              straight_count = 0;
              sr.set_action(SearchRun::TURN_RIGHT_90);
              break;
          }
          d = nextDir;
        }
        if (straight_count) sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
        straight_count = 0;
        searchAlgorithm.updateCurVecDir(Vector(0, 0), Dir::North);
        sr.set_action(SearchRun::START_INIT);
        sr.enable();
        sr.waitForEnd();
        sr.disable();
        bz.play(Buzzer::CANCEL);
      }
    }
    void readyToStartWait(const int wait_ms = 1000) {
      delay(1000);
      for (int ms = 0; ms < wait_ms; ms++) {
        delay(1);
        if (fabs(imu.accel.z) > 9800 * 1) {
          bz.play(Buzzer::CANCEL);
          waitForever();
        }
      }
    }
    void waitForever() {
      delay(100);
      isRunningFlag = false;
      while (1) delay(1000);
    }
    virtual void task() {
      maze = maze_backup.back();
      searchAlgorithm.reset();
      if (searchAlgorithm.getState() != SearchAlgorithm::REACHED_START) {
        if (!searchAlgorithm.calcShortestDirs() || isForceSearch) {
          maze = maze_backup.front();
          searchAlgorithm.reset();
          if (!searchRun()) waitForever();
          readyToStartWait(1000);
        }
      }
      if (!searchAlgorithm.calcShortestDirs()) {
        printf("Couldn't solve the maze!\n");
        bz.play(Buzzer::ERROR);
        waitForever();
      }
      while (1) {
        fast_run();
        fr.runParameter.curve_gain *= 1.05f;
        fr.runParameter.max_speed *= 1.2;
        fr.runParameter.accel *= 1.05f;
        fr.runParameter.decel *= 1.05f;
        readyToStartWait();
      }
    }
};

