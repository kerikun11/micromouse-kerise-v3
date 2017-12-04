#pragma once

#include <Arduino.h>
#include "config.h"

#include "buzzer.h"
#include "led.h"
#include "button.h"
#include "motor.h"
#include "imu.h"
#include "MazeSolver.h"

#define EMERGENCY_TASK_PRIORITY 4
#define EMERGENCY_STACK_SIZE    2048

class Emergency {
  public:
    Emergency() {}
    void begin() {
      xTaskCreate([](void* obj) {
        static_cast<Emergency*>(obj)->task();
      }, "Emergency", EMERGENCY_STACK_SIZE, this, EMERGENCY_TASK_PRIORITY, NULL);
    }
  private:
    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        if (fabs(imu.accel.y) > 9807 * 12 || fabs(imu.gyro.z) > 180.0f * PI * 1800) {
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

