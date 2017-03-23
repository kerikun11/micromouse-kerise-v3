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
extern Motor mt;
extern Fan fan;
extern MPU6500 mpu;
extern Reflector ref;
extern WallDetector wd;
extern SpeedController sc;
extern MoveAction ma;

#define LOGGER_TASK_PRIORITY 2
#define LOGGER_STACK_SIZE    4096

class Logger: TaskBase {
  public:
    Logger(): TaskBase("Logger Task", LOGGER_TASK_PRIORITY, LOGGER_STACK_SIZE) {}
    virtual ~Logger() {}
    void start() {
      log = "";
      create_task();
    }
    void end() {
      delete_task();
    }
    void print() {
      Serial.print(log);
    }
  private:
    String log;
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 2 / portTICK_RATE_MS);
        char str[64];
        const int i = 1;
        sprintf(str, "%.0f,%.0f,%.0f,%.0f,%.0f\n", sc.actual.wheel[i], sc.target.wheel[i], sc.Kp * (sc.target.wheel[i] - sc.actual.wheel[i]), sc.Kp * sc.Ki * (0 - sc.integral.wheel[i]), sc.Kp * sc.Kd * (0 - sc.differential.wheel[i]));
        //        const int num = 10;
        //        static int data[num];
        //        for (int j = num-2; j >0; j++) {
        //          data[j + 1] = data[j];
        //        }
        //        data[0] = as.getPulses(i);
        //        int now = as.getPulses(i);
        //        int sum = 0;
        //        for (int j = 0; j < num; j++) {
        //          sum += data[j] / pow(2, j);
        //        }
        //        sum /= 2;
        //        static int prev, prev_sum;
        //        sprintf(str, "%d,%d\n", now - prev, sum - prev_sum);
        //        prev = now;
        //        prev_sum = sum;
        log += str;
      }
    }
};

