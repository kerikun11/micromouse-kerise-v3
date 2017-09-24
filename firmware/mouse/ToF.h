#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "TaskBase.h"
#include "config.h"
#include "VL53L0X.h"

#define TOF_SDA_PIN           21
#define TOF_SCL_PIN           22

#define TOF_TASK_PRIORITY     1
#define TOF_TASK_STACK_SIZE   4096

class ToF : private TaskBase {
  public:
    ToF(): TaskBase("ToF", TOF_TASK_PRIORITY, TOF_TASK_STACK_SIZE) {}
    virtual ~ToF() {}
    void init() {
      Wire.begin();
      sensor.init();
      sensor.setTimeout(500);
      sensor.setAddress(0x55);
      create_task();
    }
    uint16_t getDistance() {
      return distance;
    }
    void print() {
      printf("ToF: %d\n", getDistance());
    }
  private:
    VL53L0X sensor;
    uint16_t distance;
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        uint16_t value = sensor.readRangeContinuousMillimeters();
        if (value != 8190) {
          distance = value;
        }
      }
    }
};

