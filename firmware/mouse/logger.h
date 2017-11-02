#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"
#include "SpeedController.h"

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
        char str[64];
        const int i = 0;
        //        snprintf(str, 64, "%.0f,%.0f,%.0f,%.0f,%.0f,%f\n",
        //                 sc.actual.wheel[i], sc.target.wheel[i],
        //                 sc.Kp * (sc.target.wheel[i] - sc.actual.wheel[i]),
        //                 sc.Ki * sc.integral.wheel[i],
        //                 sc.Kd * sc.differential.wheel[i],
        //                 icm.accel.y / 100);
        //        snprintf(str,64, "%.0f,%.0f,%.0f,%.0f,%.0f\n", sc.actual.wheel[i], sc.target.wheel[i], sc.Kp * (sc.target.wheel[i] - sc.actual.wheel[i]), sc.Ki * sc.integral.wheel[i], sc.Kd * sc.differential.wheel[i]);
        //        snprintf(str, 64, "%f,%f,%f,%f,%f,%f\n",
        //                 sc.target.trans,
        //                 sc.actual.trans,
        //                 sc.Kp * sc.proportional.trans,
        //                 sc.Ki * sc.integral.trans,
        //                 sc.Kd * sc.differential.trans,
        //                 icm.angle.z * 100
        //                );
        //        snprintf(str, 64, "%f,%f,%f,%f,%f,%f\n",
        //                 sc.target.rot,
        //                 sc.actual.rot,
        //                 sc.Kp * sc.proportional.rot,
        //                 sc.Ki * sc.integral.rot,
        //                 sc.Kd * sc.differential.rot,
        //                 icm.angle.z
        //                );
        snprintf(str, 64, "%f,%f,%f\n",
                 sc.actual.rot,
                 icm.gyro.z,
                 icm.angle.z
                );
        log += str;
      }
    }
};

extern Logger lg;

