#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"

#include "as5145.h"
#include "UserInterface.h"
#include "Emergency.h"
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
extern Emergency em;
extern Logger lg;
extern Motor mt;
extern Fan fan;
extern MPU6500 mpu;
extern Reflector ref;
extern WallDetector wd;
extern SpeedController sc;

#define EXTERNAL_CONTROLLER_TASK_PRIORITY 1
#define EXTERNAL_CONTROLLER_STACK_SIZE    4096

class ExternalController: TaskBase {
  public:
    ExternalController(): TaskBase("External Controller Task", EXTERNAL_CONTROLLER_TASK_PRIORITY, EXTERNAL_CONTROLLER_STACK_SIZE) {}
    virtual ~ExternalController() {}
    void init() {
      create_task();
    }
  private:
    //    WiFiClient client;
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);
//        if (Serial.available()) {
//          String s = Serial.readString();
//          s.trim();
//          sc.Kp = s.toFloat();
//          printf("Kp is set to %f\n", sc.Kp);
//        }
      }
    }
};

