#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"

#include "Reflector.h"

#define LOGGER_TASK_PRIORITY 1
#define LOGGER_STACK_SIZE    4096

class Logger: TaskBase {
  public:
    Logger(): TaskBase("Logger", LOGGER_TASK_PRIORITY, LOGGER_STACK_SIZE) {}
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
    void printf(const char* format, ...) {
      const int buf_size = 64;
      char s[buf_size];
      va_list args;
      va_start(args, format);
      vsnprintf(s, buf_size, format, args);
      va_end(args);
      Serial.print(s);
      log += String(millis(), DEC) + ": " + s;
    }
  private:
    String log;
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 5 / portTICK_RATE_MS);
        //        char str[64];
        //        const int i = 0;
        //        sprintf(str, "%.0f,%.0f,%.0f,%.0f,%.0f\n", sc.actual.wheel[i], sc.target.wheel[i], sc.Kp * (sc.target.wheel[i] - sc.actual.wheel[i]), sc.Kp * sc.Ki * (0 - sc.integral.wheel[i]), sc.Kp * sc.Kd * (0 - sc.differential.wheel[i]));
        //        log += str;
        printf("0,1800,%d,%d,%d,%d\n", ref.side(0), ref.front(0), ref.front(1), ref.side(1));
      }
    }
};

extern Logger lg;

