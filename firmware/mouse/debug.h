#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"

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
#include "MoveAction.h"

extern AS5145 as;
extern Buzzer bz;
extern Button btn;
extern LED led;
extern Logger lg;
extern Motor mt;
extern Fan fan;
extern MPU6500 mpu;
extern Reflector ref;
extern WallDetector wd;
extern SpeedController sc;
extern MoveAction ma;

#define EXTERNAL_CONTROLLER_TASK_PRIORITY 2
#define EXTERNAL_CONTROLLER_STACK_SIZE    2048

class ExternalController: TaskBase {
  public:
    ExternalController(): TaskBase("External Controller Task", EXTERNAL_CONTROLLER_TASK_PRIORITY, EXTERNAL_CONTROLLER_STACK_SIZE) {}
    virtual ~ExternalController() {}
    void init() {
      create_task();
    }
  private:
    WiFiClient client;
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);
        //        continue;
        /*
          if (!client.connected() && client.connect("192.168.11.9", 1234)) {
          printf("Connection Failed\n");
          continue;
          }
          if (client.available()) {
          bz.play(Buzzer::CONFIRM);
          String s = client.readStringUntil(',');
          s.trim();
          sc.Kp = s.toFloat();
          s = client.readStringUntil(',');
          s.trim();
          sc.Ki = s.toFloat();
          s = client.readStringUntil('\r');
          s.trim();
          sc.Kd = s.toFloat();
          char str[64];
          sprintf(str, "Kp: %f\tKi: %f\tKd: %f\n", sc.Kp, sc.Ki, sc.Kd);
          client.print(str);
          client.flush();
          }
        */
        //        /*
        while (Serial.available()) {
          char c = Serial.read();
          printf("%c\n", c);
          switch (c) {
            case 't':
              bz.play(Buzzer::CONFIRM);
              mpu.calibration();
              wd.calibration();
              break;
            case 'g':
              bz.play(Buzzer::CONFIRM);
              mpu.calibration();
              wd.calibration();
              ma.enable();
              break;
            case 'f':
              bz.play(Buzzer::CANCEL);
              ma.disable();
              break;
            case 's':
              ma.set_action(MoveAction::START_STEP);
              bz.play(Buzzer::SELECT);
              break;
            case 'i':
              ma.set_action(MoveAction::START_INIT);
              bz.play(Buzzer::SELECT);
              break;
            case 'w':
              ma.set_action(MoveAction::GO_STRAIGHT);
              bz.play(Buzzer::SELECT);
              break;
            case 'a':
              ma.set_action(MoveAction::TURN_LEFT_90);
              bz.play(Buzzer::SELECT);
              break;
            case 'd':
              ma.set_action(MoveAction::TURN_RIGHT_90);
              bz.play(Buzzer::SELECT);
              break;
            case 'b':
              ma.set_action(MoveAction::STOP);
              bz.play(Buzzer::SELECT);
              break;
            case 'r':
              ma.set_action(MoveAction::RETURN);
              bz.play(Buzzer::SELECT);
              break;
            case 'z':
              ma.set_action(MoveAction::FAST_TURN_LEFT_90);
              bz.play(Buzzer::SELECT);
              break;
            case 'x':
              ma.set_action(MoveAction::FAST_GO_STRAIGHT);
              bz.play(Buzzer::SELECT);
              break;
            case 'c':
              ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
              bz.play(Buzzer::SELECT);
              break;
            case 'u':
              ma.set_action(MoveAction::START_STEP);
              for (int i = 0; i < 5; i++) {
                ma.set_action(MoveAction::TURN_RIGHT_90);
                ma.set_action(MoveAction::TURN_RIGHT_90);
                ma.set_action(MoveAction::RETURN);
                ma.set_action(MoveAction::TURN_LEFT_90);
                ma.set_action(MoveAction::TURN_LEFT_90);
                ma.set_action(MoveAction::RETURN);
              }
              ma.set_action(MoveAction::TURN_RIGHT_90);
              ma.set_action(MoveAction::TURN_RIGHT_90);
              ma.set_action(MoveAction::RETURN);
              ma.set_action(MoveAction::TURN_LEFT_90);
              ma.set_action(MoveAction::TURN_LEFT_90);
              ma.set_action(MoveAction::START_INIT);
              bz.play(Buzzer::CONFIRM);
              mpu.calibration();
              wd.calibration();
              ma.enable();
              break;
            case 'm':
              //              bz.play(Buzzer::CONFIRM);
              //              ms->terminate();
              //              ms->start();
              break;
            case 'p':
              bz.play(Buzzer::SELECT);
              wd.print();
              break;
          }
        }
        //        */
      }
    }
};

