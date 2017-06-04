#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"

#include <Agent.h>
#include <Maze.h>
#include <mazeData.h>
#include <MazeSolver_conf.h>
#include <Operation.h>
#include <ShortestPath.h>
#include <vector>

#include "as5145.h"
#include "UserInterface.h"
#include "Emergency.h"
#include "debug.h"
#include "logger.h"
#include "motor.h"
#include "mpu6500.h"
#include "reflector.h"
#include "WallDetector.h"
#include "SpeedController.h"
#include "FastRun.h"
#include "SearchRun.h"

#define MAZE_SOLVER_TASK_PRIORITY 2
#define MAZE_SOLVER_STACK_SIZE    8092

//#define printf  lg.printf

class MazeSolver: TaskBase {
  public:
    MazeSolver(): TaskBase("Maze Solver", MAZE_SOLVER_TASK_PRIORITY, MAZE_SOLVER_STACK_SIZE), agent(maze) {
      dir = NORTH;
      pos = IndexVec(0, 0);
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
    void printWall() {
      maze.printWall();
      printf("State: %d\n", agent.getState());
    }
  private:
    Maze maze, maze_backup;
    Agent agent;
    Agent::State prevState = Agent::IDLE;
    Direction dir;
    IndexVec pos;

    void robotMove(const Direction &nextDir) {
      if (dir == NORTH) {
        if (nextDir == NORTH) {
          sr.set_action(SearchRun::GO_STRAIGHT);
          pos.y++;
          dir = NORTH;
        } else if (nextDir == EAST) {
          sr.set_action(SearchRun::TURN_RIGHT_90);
          pos.x++;
          dir = EAST;
        } else if (nextDir == SOUTH) {
          sr.set_action(SearchRun::TURN_BACK);
          pos.y--;
          dir = SOUTH;
        } else if (nextDir == WEST) {
          sr.set_action(SearchRun::TURN_LEFT_90);
          pos.x--;
          dir = WEST;
        }
      } else if (dir == EAST) {
        if (nextDir == NORTH) {
          sr.set_action(SearchRun::TURN_LEFT_90);
          pos.y++;
          dir = NORTH;
        } else if (nextDir == EAST) {
          sr.set_action(SearchRun::GO_STRAIGHT);
          pos.x++;
          dir = EAST;
        } else if (nextDir == SOUTH) {
          sr.set_action(SearchRun::TURN_RIGHT_90);
          pos.y--;
          dir = SOUTH;
        } else if (nextDir == WEST) {
          sr.set_action(SearchRun::TURN_BACK);
          pos.x--;
          dir = WEST;
        }
      } else if (dir == SOUTH) {
        if (nextDir == NORTH) {
          sr.set_action(SearchRun::TURN_BACK);
          pos.y++;
          dir = NORTH;
        } else if (nextDir == EAST) {
          sr.set_action(SearchRun::TURN_LEFT_90);
          pos.x++;
          dir = EAST;
        } else if (nextDir == SOUTH) {
          sr.set_action(SearchRun::GO_STRAIGHT);
          pos.y--;
          dir = SOUTH;
        } else if (nextDir == WEST) {
          sr.set_action(SearchRun::TURN_RIGHT_90);
          pos.x--;
          dir = WEST;
        }
      } else if (dir == WEST) {
        if (nextDir == NORTH) {
          sr.set_action(SearchRun::TURN_RIGHT_90);
          pos.y++;
          dir = NORTH;
        } else if (nextDir == EAST) {
          sr.set_action(SearchRun::TURN_BACK);
          pos.x++;
          dir = EAST;
        } else if (nextDir == SOUTH) {
          sr.set_action(SearchRun::TURN_LEFT_90);
          pos.y--;
          dir = SOUTH;
        } else if (nextDir == WEST) {
          sr.set_action(SearchRun::GO_STRAIGHT);
          pos.x--;
          dir = WEST;
        }
      }
    }
    Direction getWallData() {
      Direction wall;
      if (dir == NORTH) {
        if (wd.wall().side[0]) {
          wall |= WEST;
        }
        if (wd.wall().side[1]) {
          wall |= EAST;
        }
        if (wd.wall().front[0] && wd.wall().front[1]) {
          wall |= NORTH;
        }
        wall |= DONE_NORTH;
        wall |= DONE_EAST;
        wall |= DONE_SOUTH;
        wall |= DONE_WEST;
      } else if (dir == EAST) {
        if (wd.wall().side[0]) {
          wall |= NORTH;
        }
        if (wd.wall().side[1]) {
          wall |= SOUTH;
        }
        if (wd.wall().front[0] && wd.wall().front[1]) {
          wall |= EAST;
        }
        wall |= DONE_NORTH;
        wall |= DONE_EAST;
        wall |= DONE_SOUTH;
        wall |= DONE_WEST;
      } else if (dir == SOUTH) {
        if (wd.wall().side[0]) {
          wall |= EAST;
        }
        if (wd.wall().side[1]) {
          wall |= WEST;
        }
        if (wd.wall().front[0] && wd.wall().front[1]) {
          wall |= SOUTH;
        }
        wall |= DONE_NORTH;
        wall |= DONE_EAST;
        wall |= DONE_SOUTH;
        wall |= DONE_WEST;
      } else if (dir == WEST) {
        if (wd.wall().side[0]) {
          wall |= SOUTH;
        }
        if (wd.wall().side[1]) {
          wall |= NORTH;
        }
        if (wd.wall().front[0] && wd.wall().front[1]) {
          wall |= WEST;
        }
        wall |= DONE_NORTH;
        wall |= DONE_EAST;
        wall |= DONE_SOUTH;
        wall |= DONE_WEST;
      }
      return wall;
    }
    void search_run() {
      maze = maze_backup;
      dir = NORTH;
      pos = IndexVec(0, 0);
      Direction wallData = 0xFE;
      agent.update(pos, wallData);

      if (agent.getState() == Agent::FINISHED || agent.getState() == Agent::BACK_TO_START) return;

      sr.set_action(SearchRun::START_STEP);
      pos = IndexVec(0, 1);

      mpu.calibration(false);
      wd.calibration();
      mpu.calibrationWait();
      bz.play(Buzzer::CONFIRM);
      sr.enable();
      while (1) {
        sr.waitForEnd();

        delay(500); /* debug */
        Direction wallData = getWallData();
        printf("Vec:\t(%d, %d)\tWall:\t0x%X\n", pos.x, pos.y, (int)wallData);

        agent.update(pos, wallData);
        printf("State: %d\n", agent.getState());
        if (agent.getState() == Agent::FINISHED) {
          // スタートまで戻って来て，探索終了
          maze_backup = maze;
          break;
        }
        if (prevState != Agent::SEARCHING_REACHED_GOAL && agent.getState() == Agent::SEARCHING_REACHED_GOAL) {
          // 追加探索
          printf("maze_backup 1\n");
          maze_backup = maze;
          bz.play(Buzzer::CONFIRM);
        }
        if (prevState != Agent::BACK_TO_START && agent.getState() == Agent::BACK_TO_START) {
          // スタートへ戻る
          printf("maze_backup 2\n");
          maze_backup = maze;
          bz.play(Buzzer::COMPLETE);
        }
        prevState = agent.getState();

        Direction nextDir = agent.getNextDirection();     //< Agentの状態が探索中の場合は次に進むべき方向を取得する
        //        delay(100); /* debug */
        printf("NextDir: %X\n", (int) nextDir);
        if (nextDir == 0) {
          bz.play(Buzzer::ERROR);
          sr.set_action(SearchRun::STOP);
          sr.waitForEnd();
          sr.disable();
          while (1) {
            delay(1000);
          }
        }
        robotMove(nextDir);  //< robotMove関数はDirection型を受け取ってロボットをそっちに動かす関数
      }
      sr.set_action(SearchRun::START_INIT);
      sr.waitForEnd();
      sr.disable();
      bz.play(Buzzer::COMPLETE);
    }
    void fast_run() {
      maze = maze_backup;
      printf("agent.calcRunSequence();\n");
      agent.calcRunSequence(false);
      const OperationList &runSequence = agent.getRunSequence();
      printf("runSequence.size() => %d\n", runSequence.size());
      if (runSequence.size() == 0) {
        printf("Couldn't solve the maze!\n");
        bz.play(Buzzer::ERROR);
        return;
      }
      for (size_t i = 0; i < runSequence.size(); i++) {
        printf("runSequence[%d].n => %d, runSequence[%d].op => %d\n", i, runSequence[i].n, i, runSequence[i].op);
        const Operation& op = runSequence[i];
        if (i == 0) {
          fr.set_action(FastRun::FAST_GO_STRAIGHT, op.n - 1);
        } else {
          switch (op.op) {
            case Operation::FORWARD:
              fr.set_action(FastRun::FAST_GO_STRAIGHT, op.n);
              break;
            case Operation::TURN_LEFT90:
              fr.set_action(FastRun::FAST_TURN_LEFT_90, op.n);
              break;
            case Operation::TURN_RIGHT90:
              fr.set_action(FastRun::FAST_TURN_RIGHT_90, op.n);
              break;
            case Operation::FORWARD_DIAG:
            case Operation::TURN_LEFT45:
            case Operation::TURN_RIGHT45:
            case Operation::STOP:
              break;
          }
        }
      }

      // start drive
      bz.play(Buzzer::CONFIRM);
      mpu.calibration();
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
      for (size_t i = 0; i < runSequence.size(); i++) {
        const Operation& op = runSequence[runSequence.size() - 1 - i];
        if (i == runSequence.size() - 1) {
          sr.set_action(SearchRun::GO_STRAIGHT, op.n - 1);
        } else {
          switch (op.op) {
            case Operation::FORWARD:
              sr.set_action(SearchRun::GO_STRAIGHT, op.n);
              break;
            case Operation::TURN_LEFT90:
              sr.set_action(SearchRun::TURN_RIGHT_90, op.n);
              break;
            case Operation::TURN_RIGHT90:
              sr.set_action(SearchRun::TURN_LEFT_90, op.n);
              break;
            case Operation::TURN_LEFT45:
            case Operation::TURN_RIGHT45:
            case Operation::FORWARD_DIAG:
            case Operation::STOP:
              break;
          }
        }
      }
      sr.set_action(SearchRun::START_INIT);
      sr.enable();
      sr.waitForEnd();
      sr.disable();
      bz.play(Buzzer::CANCEL);
    }
    virtual void task() {
      delay(500);
      search_run();
      delay(2000);
      fast_run();
      while (1) {
        delay(1000);
      }
    }
};

extern MazeSolver ms;

