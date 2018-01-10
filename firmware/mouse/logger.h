#pragma once

#include <Arduino.h>
#include "config.h"

/* Hardware */
#include "imu.h"
extern IMU imu;
#include "encoder.h"
extern Encoder enc;
#include "reflector.h"
extern Reflector ref;
#include "tof.h"
extern ToF tof;

#define LOGGER_TASK_PRIORITY 1
#define LOGGER_STACK_SIZE    4096

class Logger {
  public:
    Logger() {}
    void start() {
      log = "";
      xTaskCreate([](void* obj) {
        static_cast<Logger*>(obj)->task();
      }, "Logger", LOGGER_STACK_SIZE, this, LOGGER_TASK_PRIORITY, &task_handle);
    }
    void end() {
      vTaskDelete(&task_handle);
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
      //      log += String(millis(), DEC) + ": " + s;
      log += s;
    }
  private:
    xTaskHandle task_handle;
    String log;
    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 2 / portTICK_RATE_MS);
        char str[64];
        //        const int i = 0;
        //        sprintf(str, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", sc.target.wheel[i], sc.actual.wheel[i], sc.enconly.wheel[i], sc.Kp * (sc.target.wheel[i] - sc.actual.wheel[i]), sc.Ki * (0 - sc.integral.wheel[i]), sc.Kd * (0 - sc.differential.wheel[i]));
        //        snprintf(str, 64, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
        //                 sc.target.trans,
        //                 sc.actual.trans,
        //                 sc.enconly.trans,
        //                 sc.Kp * sc.proportional.trans,
        //                 sc.Ki * sc.integral.trans,
        //                 sc.Kd * sc.differential.trans,
        //                 sc.Kp * sc.proportional.trans + sc.Ki * sc.integral.trans + sc.Kd * sc.differential.trans
        //                );
        //        snprintf(str, 64, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
        //                 sc.target.rot,
        //                 sc.actual.rot,
        //                 sc.enconly.rot,
        //                 sc.Kp * sc.proportional.rot,
        //                 sc.Ki * sc.integral.rot,
        //                 sc.Kd * sc.differential.rot,
        //                 sc.Kp * sc.proportional.rot + sc.Ki * sc.integral.rot + sc.Kd * sc.differential.rot
        //                );
        //        sprintf(str, "%f,%f,%f,%f\n", sc.position.x, sc.position.y, sc.target.trans, sc.target.rot);
        //        sprintf(str, "%f,%f,%f\n", sc.position.x, sc.position.y, sc.position.theta);
        snprintf(str, 64, "%d,%f,%f,%f,%f\n",
                 sc.ave_num * 100,
                 sc.target.trans,
                 sc.actual.trans,
                 sc.acconly.trans,
                 sc.enconly.trans
                );
        log += str;
      }
    }
};

