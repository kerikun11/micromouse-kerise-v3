#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "VL53L0X.h"

#define TOF_TASK_PRIORITY     1
#define TOF_TASK_STACK_SIZE   4096

class ToF {
  public:
    ToF(const int pin_sda, const int pin_scl): pin_sda(pin_sda), pin_scl(pin_scl) {}
    void begin() {
      Wire.begin(pin_sda, pin_scl);
      sensor.setTimeout(500);
      sensor.init();
      //      sensor.setAddress(0x55);
      sensor.setMeasurementTimingBudget(20000);
      sensor.startContinuous();
      xTaskCreate([](void* obj) {
        static_cast<ToF*>(obj)->task();
      }, "ToF", TOF_TASK_STACK_SIZE, this, TOF_TASK_PRIORITY, NULL);
    }
    uint16_t getDistance() {
      return distance;
    }
    void print() {
      log_d("ToF: %d\n", getDistance());
    }
    void csv() {
      printf("0,90,180,270,360,%d\n", getDistance());
    }
  private:
    const int pin_sda, pin_scl;
    VL53L0X sensor;
    uint16_t distance;
    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        uint16_t value = sensor.readRangeContinuousMillimeters();
        if (value < 2000) {
          distance = value;
        }
      }
    }
};

extern ToF tof;
