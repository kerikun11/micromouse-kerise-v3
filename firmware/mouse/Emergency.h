#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"

#include "UserInterface.h"
#include "motor.h"
#include "mpu6500.h"

extern Buzzer bz;
extern Button btn;
extern LED led;
extern Motor mt;
extern Fan fan;
extern MPU6500 mpu;

#define EMERGENCY_TASK_PRIORITY 4
#define EMERGENCY_STACK_SIZE    2048

class Emergency: TaskBase {
  public:
    Emergency(): TaskBase("Emergency Task", EMERGENCY_TASK_PRIORITY, EMERGENCY_STACK_SIZE), emergency_flag(false) {}
    virtual ~Emergency() {}
    void init() {
      create_task();
    }
    void release() {
      emergency_flag = false;
    }
    bool isEmergency() {
      return emergency_flag;
    }
  private:
    bool emergency_flag;
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        if (fabs(mpu.accel.y) > 9800 * 5) {
          mt.emergency_stop();
          fan.drive(0);
          bz.play(Buzzer::EMERGENCY);
          while (1) {
            vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
            if (btn.pressed) {
              btn.flags = 0;
              bz.play(Buzzer::BOOT);
            }
          }
        }
      }
    }
};

