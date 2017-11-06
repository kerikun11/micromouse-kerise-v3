#pragma once

#include <Arduino.h>
#include <SPIFFS.h>
#include "TaskBase.h"
#include "config.h"
#include "Maze.h"
#include "Agent.h"

#include "encoder.h"
#include "UserInterface.h"
#include "Emergency.h"
#include "debug.h"
#include "logger.h"
#include "motor.h"
#include "axis.h"
#include "reflector.h"
#include "WallDetector.h"
#include "SpeedController.h"
#include "FastRun.h"
#include "SearchRun.h"

#define MAZE_SOLVER_TASK_PRIORITY 2
#define MAZE_SOLVER_STACK_SIZE    8192

//#define MAZE_GOAL {Vector(7,7), Vector(7,8), Vector(8,7), Vector(8,8)}
//#define MAZE_GOAL {{Vector(5, 6), Vector(5, 7), Vector(6, 6), Vector(6, 7)}}
#define MAZE_GOAL {Vector(1,0)}
#define MAZE_BACKUP_SIZE 5

//#define printf  lg.printf

#define MAZE_BACKUP_PATH    "/maze_backup.maze"

class MazeSolver: TaskBase {
  public:
    MazeSolver(): TaskBase("Maze Solver", MAZE_SOLVER_TASK_PRIORITY, MAZE_SOLVER_STACK_SIZE), agent(maze, MAZE_GOAL) {
      maze_backup.push_back(maze);
    }
    virtual ~MazeSolver() {}
    void start() {
      terminate();
      create_task();
    }
    void terminate() {
      delete_task();
      sr.disable();
      fr.disable();
    }
    void print() {
      int i = 0;
      for (auto& maze : maze_backup) {
        printf("Backup Maze %d:\n", i++);
        maze.print();
      }
      agent.printInfo();
    }
    bool isRunning() {
      return handle != NULL;
    }
    void set_goal(const std::vector<Vector>& goal) {
      agent.reset(goal);
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
      agent.reset();
      return true;
    }
  private:
    Maze maze;
    std::deque<Maze> maze_backup;
    Agent agent;

    bool search_run() {
      sr.set_action(SearchRun::START_STEP);
      agent.updateCurVecDir(Vector(0, 1), Dir::North);
      sr.enable();
      Agent::State prevState = agent.getState();
      while (1) {
        uint32_t us;

        //        us = micros();
        //        backup();
        //        printf("backup(); %ld [us]\n", micros() - us);

        sr.waitForEnd();

        //        delay(100); // センサが安定するのを待つ


        const Vector& v = agent.getCurVec();
        const Dir& d = agent.getCurDir();
        printf("Cur: ( %3d, %3d, %3d), State: %s       \n", v.x, v.y, uint8_t(d), agent.stateString(agent.getState()));
        agent.updateWall(v, d + 1, wd.getWall(0)); // left
        agent.updateWall(v, d + 0, wd.getWall(2)); // front
        agent.updateWall(v, d - 1, wd.getWall(1)); // right

        us = micros();
        agent.calcNextDir();
        printf("agent.calcNextDir(); %ld [us]\n", micros() - us);
        Agent::State newState = agent.getState();
        if (newState != prevState && newState == Agent::REACHED_START) break;
        if (newState != prevState && newState == Agent::REACHED_GOAL) {
          /* REACHED_GOAL */
          bz.play(Buzzer::CONFIRM);
        }
        if (newState != prevState && newState == Agent::SEARCHING_ADDITIONALLY) {
          /* SEARCHING_ADDITIONALLY */
          bz.play(Buzzer::CONFIRM);
        }
        if (newState != prevState && newState == Agent::BACKING_TO_START) {
          /* BACKING_TO_START */
          bz.play(Buzzer::COMPLETE);
        }
        prevState = newState;
        auto nextDirs = agent.getNextDirs();
        if (nextDirs.empty()) {
          bz.play(Buzzer::ERROR);
          sr.set_action(SearchRun::STOP);
          sr.waitForEnd();
          sr.disable();
          backup();
          return false;
        }
        int straight_count = 0;
        for (auto nextDir : nextDirs) {
          Vector nextVec = agent.getCurVec().next(nextDir);
          switch (Dir(nextDir - agent.getCurDir())) {
            case Dir::East:
              straight_count++;
              break;
            case Dir::North:
              if (straight_count) sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
              straight_count = 0;
              sr.set_action(SearchRun::TURN_LEFT_90);
              break;
            case Dir::West:
              if (straight_count) sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
              straight_count = 0;
              sr.set_action(SearchRun::TURN_BACK);
              break;
            case Dir::South:
              if (straight_count) sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
              straight_count = 0;
              sr.set_action(SearchRun::TURN_RIGHT_90);
              break;
          }
          agent.updateCurVecDir(nextVec, nextDir);
        }
        if (straight_count) sr.set_action(SearchRun::GO_STRAIGHT, straight_count);
        straight_count = 0;
        maze_backup.push_back(maze);
        if (maze_backup.size() > MAZE_BACKUP_SIZE) maze_backup.pop_front();
      }
      if (agent.getState() != Agent::REACHED_START) return false;
      sr.set_action(SearchRun::START_INIT);
      sr.waitForEnd();
      sr.disable();
      if (!agent.calcShortestDirs()) {
        printf("Couldn't solve the maze!\n");
        bz.play(Buzzer::ERROR);
        return false;
      }
      bz.play(Buzzer::COMPLETE);
      return true;
    }
    void fast_run() {
      auto path = agent.getShortestDirs();
      path.erase(path.begin());
      Dir prev_dir = Dir::North;
      for (auto nextDir : path) {
        switch (Dir(nextDir - prev_dir)) {
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
        prev_dir = nextDir;
      }

      // start drive
      bz.play(Buzzer::CONFIRM);
      axis.calibration();
      fr.enable();
      fr.waitForEnd();
      fr.disable();
      // end drive
      readyToStartWait(2000);

      // back to start
      printf("Back to Start\n");
      sr.setPosition();
      sr.set_action(SearchRun::RETURN);
      sr.set_action(SearchRun::GO_HALF);
      path = agent.getShortestDirs();
      prev_dir = path.back();
      path.pop_back();
      std::reverse(path.begin(), path.end());
      for (auto nextDir : path) {
        switch (Dir(nextDir - prev_dir)) {
          case Dir::East:
            sr.set_action(SearchRun::GO_STRAIGHT);
            break;
          case Dir::North:
            sr.set_action(SearchRun::TURN_LEFT_90);
            break;
          case Dir::West:
            break;
          case Dir::South:
            sr.set_action(SearchRun::TURN_RIGHT_90);
            break;
        }
        prev_dir = nextDir;
      }
      sr.set_action(SearchRun::START_INIT);
      sr.enable();
      sr.waitForEnd();
      sr.disable();
      bz.play(Buzzer::CANCEL);
    }
    void readyToStartWait(const int wait_ms = 3000) {
      for (int ms = 0; ms < wait_ms; ms++) {
        delay(1);
        if (fabs(axis.accel.z) > 9800 * 1) {
          bz.play(Buzzer::CANCEL);
          while (1) delay(1000);
        }
      }
    }
    virtual void task() {
      bz.play(Buzzer::CONFIRM);
      axis.calibration(false);
      wd.calibration();
      axis.calibrationWait();
      bz.play(Buzzer::CANCEL);

      maze = maze_backup.back();
      agent.reset();
      if (agent.getState() != Agent::REACHED_START) {
        maze = maze_backup.front();
        agent.reset();
        if (!search_run()) {
          while (1) delay(1000);
        }
        backup();
        readyToStartWait();
      }

      if (!agent.calcShortestDirs()) {
        printf("Couldn't solve the maze!\n");
        bz.play(Buzzer::ERROR);
        while (1) delay(1000);
      }
      while (1) {
        fast_run();
        fr.fast_speed *= 1.1;
        fr.fast_curve_gain *= 1.1;
        readyToStartWait();
        bz.play(Buzzer::CONFIRM);
        axis.calibration();
        bz.play(Buzzer::CANCEL);
      }
    }
};

extern MazeSolver ms;

