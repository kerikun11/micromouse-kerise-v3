#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"
#include "Maze.h"

#include "UserInterface.h"
#include "Emergency.h"
#include "logger.h"
#include "motor.h"
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

class MazeSolver: TaskBase {
  public:
    MazeSolver(): TaskBase("Maze Solver", MAZE_SOLVER_TASK_PRIORITY, MAZE_SOLVER_STACK_SIZE), agent(maze, MAZE_GOAL) {
      maze_backup.push(maze);
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
      agent.printInfo();
    }
    bool isRunning() {
      return handle != NULL;
    }
    void set_goal(const std::vector<Vector>& goal) {
      agent.reset(goal);
    }
  private:
    Maze maze;
    std::queue<Maze> maze_backup;
    Agent agent;

    bool search_run() {
      maze = maze_backup.back();
      agent.reset();
      if (agent.getState() == Agent::REACHED_START) return true;
      maze = maze_backup.front();
      agent.reset();

      sr.set_action(SearchRun::START_STEP);
      agent.updateCurVecDir(Vector(0, 1), Dir::North);
      axis.calibration(false);
      wd.calibration();
      axis.calibrationWait();
      bz.play(Buzzer::CONFIRM);
      sr.enable();
      Agent::State prevState = agent.getState();
      while (1) {
        sr.waitForEnd();

        const Vector& v = agent.getCurVec();
        const Dir& d = agent.getCurDir();
        //        delay(300); // センサが安定するのを待つ
        uint8_t wall = wd.wallDetect();
        agent.updateWall(v, d + 1, wall & 1); // left
        agent.updateWall(v, d + 0, (wall & 6) == 6); // front
        agent.updateWall(v, d - 1, wall & 8); // right

        agent.calcNextDir();
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
        maze_backup.push(maze);
        if (maze_backup.size() > MAZE_BACKUP_SIZE) maze_backup.pop();
      }
      if (agent.getState() != Agent::REACHED_START) return false;
      sr.set_action(SearchRun::START_INIT);
      sr.waitForEnd();
      sr.disable();
      bz.play(Buzzer::COMPLETE);
      return true;
    }
    void fast_run() {
      maze = maze_backup.back();
      if (!agent.calcShortestDirs()) {
        printf("Couldn't solve the maze!\n");
        bz.play(Buzzer::ERROR);
        return;
      }
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
      delay(2000);

      // back to start
      printf("Back to Start\n");
      sr.setPosition();
      sr.set_action(SearchRun::RETURN);
      sr.set_action(SearchRun::GO_HALF);
      path = agent.getShortestDirs();
      prev_dir = path.back();
      path.pop_back();
      std::reverse(path.begin(), path.end());
      int calib = 0;
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
    virtual void task() {
      delay(500);
      if (!search_run()) {
        while (1) {
          delay(1000);
        }
      }
      delay(2000);
      while (1) {
        fast_run();
        delay(3000);
        fr.fast_speed *= 1.1;
        fr.fast_curve_gain *= 1.1;
        bz.play(Buzzer::CONFIRM);
        delay(2000);
      }
    }
};

extern MazeSolver ms;

