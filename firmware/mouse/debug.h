#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"

#include "as5145.h"
#include "UserInterface.h"
#include "motor.h"
#include "mpu6500.h"
#include "reflector.h"
#include "WallDetector.h"
#include "SpeedController.h"
#include "MazeSolver.h"

#define EXTERNAL_CONTROLLER_TASK_PRIORITY 1
#define EXTERNAL_CONTROLLER_STACK_SIZE    4096

class ExternalController: TaskBase {
  public:
    ExternalController(): TaskBase("External Controller", EXTERNAL_CONTROLLER_TASK_PRIORITY, EXTERNAL_CONTROLLER_STACK_SIZE) {}
    virtual ~ExternalController() {}
    void init() {
      create_task();
    }
  private:
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);
        //        while (Serial.available()) {
        //          char c = Serial.read();
        //          printf("%c\n", c);
        //          switch (c) {
        //            case 't':
        //              bz.play(Buzzer::CONFIRM);
        //              mpu.calibration();
        //              wd.calibration();
        //              break;
        //            case 'g':
        //              bz.play(Buzzer::CONFIRM);
        //              mpu.calibration();
        //              wd.calibration();
        //              ma.enable();
        //              break;
        //            case 'f':
        //              bz.play(Buzzer::CANCEL);
        //              ma.disable();
        //              break;
        //            case 's':
        //              ma.set_action(MoveAction::START_STEP);
        //              bz.play(Buzzer::SELECT);
        //              break;
        //            case 'i':
        //              ma.set_action(MoveAction::START_INIT);
        //              bz.play(Buzzer::SELECT);
        //              break;
        //            case 'w':
        //              ma.set_action(MoveAction::GO_STRAIGHT);
        //              bz.play(Buzzer::SELECT);
        //              break;
        //            case 'a':
        //              ma.set_action(MoveAction::TURN_LEFT_90);
        //              bz.play(Buzzer::SELECT);
        //              break;
        //            case 'd':
        //              ma.set_action(MoveAction::TURN_RIGHT_90);
        //              bz.play(Buzzer::SELECT);
        //              break;
        //            case 'b':
        //              ma.set_action(MoveAction::STOP);
        //              bz.play(Buzzer::SELECT);
        //              break;
        //            case 'r':
        //              ma.set_action(MoveAction::RETURN);
        //              bz.play(Buzzer::SELECT);
        //              break;
        //            case 'z':
        //              ma.set_action(MoveAction::FAST_TURN_LEFT_90);
        //              bz.play(Buzzer::SELECT);
        //              break;
        //            case 'x':
        //              ma.set_action(MoveAction::FAST_GO_STRAIGHT);
        //              bz.play(Buzzer::SELECT);
        //              break;
        //            case 'c':
        //              ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
        //              bz.play(Buzzer::SELECT);
        //              break;
        //            case 'u':
        //              ma.set_action(MoveAction::START_STEP);
        //              for (int i = 0; i < 5; i++) {
        //                ma.set_action(MoveAction::TURN_RIGHT_90);
        //                ma.set_action(MoveAction::TURN_RIGHT_90);
        //                ma.set_action(MoveAction::RETURN);
        //                ma.set_action(MoveAction::TURN_LEFT_90);
        //                ma.set_action(MoveAction::TURN_LEFT_90);
        //                ma.set_action(MoveAction::RETURN);
        //              }
        //              ma.set_action(MoveAction::TURN_RIGHT_90);
        //              ma.set_action(MoveAction::TURN_RIGHT_90);
        //              ma.set_action(MoveAction::RETURN);
        //              ma.set_action(MoveAction::TURN_LEFT_90);
        //              ma.set_action(MoveAction::TURN_LEFT_90);
        //              ma.set_action(MoveAction::START_INIT);
        //              bz.play(Buzzer::CONFIRM);
        //              mpu.calibration();
        //              wd.calibration();
        //              ma.enable();
        //              break;
        //            case 'p':
        //              bz.play(Buzzer::SELECT);
        //              wd.print();
        //              break;
        //          }
        //        }
      }
    }
};

extern ExternalController ec;

