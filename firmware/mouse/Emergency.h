#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"

#include "UserInterface.h"
#include "motor.h"
#include "mpu6500.h"
#include "MazeSolver.h"

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
        if (fabs(mpu.accel.y) > 9800 * 9 || fabs(mpu.gyro.z) > 9 * PI) {
          mt.emergency_stop();
          fan.drive(0);
          bz.play(Buzzer::EMERGENCY);
          ms.terminate();
          delay(500);
          mt.emergency_release();
        }
      }
    }
};

extern Emergency em;

