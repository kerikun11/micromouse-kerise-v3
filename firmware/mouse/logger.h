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
    Logger() {
      log.reserve(4096);
      end();
      xTaskCreate([](void* obj) {
        static_cast<Logger*>(obj)->task();
      }, "Logger", LOGGER_STACK_SIZE, this, LOGGER_TASK_PRIORITY, &task_handle);
    }
    void start(bool clear = true) {
      if (clear) log = "";
      enabled = true;
    }
    void end() {
      enabled = false;
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
    bool enabled;
    String log;

    void printToLog() {
      char str[64];
      snprintf(str, 64, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
               sc.target.trans,
               sc.actual.trans,
               sc.enconly.trans,
               sc.Kp * sc.proportional.trans,
               sc.Ki * sc.integral.trans,
               sc.Kd * sc.differential.trans,
               sc.Kp * sc.proportional.trans + sc.Ki * sc.integral.trans + sc.Kd * sc.differential.trans
              );
      log += str;
    }

    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 2 / portTICK_RATE_MS); xLastWakeTime = xTaskGetTickCount();
        if (enabled) {
          printToLog();
        }
      }
    }
};

