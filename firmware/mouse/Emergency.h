#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"

#include "UserInterface.h"
#include "motor.h"
#include "icm20602.h"
#include "MazeSolver.h"
#include "FastRun.h"
#include "SearchRun.h"

#define EMERGENCY_TASK_PRIORITY 4
#define EMERGENCY_STACK_SIZE    2048

class Emergency: TaskBase {
  public:
    Emergency(): TaskBase("Emergency", EMERGENCY_TASK_PRIORITY, EMERGENCY_STACK_SIZE) {}
    virtual ~Emergency() {}
    void init() {
      create_task();
    }
  private:
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        if (fabs(icm.accel.y) > 9800 * 12 || fabs(icm.gyro.z) > 10 * PI) {
          mt.emergency_stop();
          fan.drive(0);
          bz.play(Buzzer::EMERGENCY);
          ms.terminate();
          sr.disable();
          fr.disable();
          delay(500);
          mt.emergency_release();
        }
      }
    }
};

extern Emergency em;

