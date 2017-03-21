#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"

#include "UserInterface.h"
#include "motor.h"
#include "mpu6500.h"
#include "SpeedController.h"
#include "MoveAction.h"

extern Buzzer bz;
extern Button btn;
extern LED led;
extern Motor mt;
extern Fan fan;
extern MPU6500 mpu;
extern SpeedController sc;

#define LOGGER_TASK_PRIORITY 1
#define LOGGER_STACK_SIZE    4096

class Logger: TaskBase {
  public:
    Logger(): TaskBase("Logger Task", LOGGER_TASK_PRIORITY, LOGGER_STACK_SIZE) {}
    virtual ~Logger() {}
    void start() {
      create_task();
    }
    void end() {
      delete_task();
    }
  private:
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);
        printf("%.1f,%.1f,%.1f\n", sc.actual.wheel[0], sc.actual.wheel[1], sc.target.wheel[1]);
        //        printf("%f,%f\n", as.position(0), as.position(1));
      }
    }
};

