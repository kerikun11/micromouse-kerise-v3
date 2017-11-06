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
      sensor.setTimeout(100);
      sensor.init();
      //      sensor.setAddress(0x55);
      sensor.setMeasurementTimingBudget(20000);
      //      sensor.startContinuous();
      xTaskCreate([](void* obj) {
        static_cast<ToF*>(obj)->task();
      }, "ToF", TOF_TASK_STACK_SIZE, this, TOF_TASK_PRIORITY, NULL);
    }
    uint16_t getDistance() {
      return distance;
    }
    uint16_t passedTimeMs() {
      return passed_ms;
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
    uint16_t passed_ms;

    void read() {
    }
    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        sensor.writeReg(0x80, 0x01);
        sensor.writeReg(0xFF, 0x01);
        sensor.writeReg(0x00, 0x00);
        sensor.writeReg(0x91, sensor.stop_variable);
        sensor.writeReg(0x00, 0x01);
        sensor.writeReg(0xFF, 0x00);
        sensor.writeReg(0x80, 0x00);
        sensor.writeReg(VL53L0X::SYSRANGE_START, 0x01);
        for (int i = 0; i < 22; i++) {
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
          passed_ms++;
        }
        uint16_t value = sensor.readRangeContinuousMillimeters();
        if (value < 2000)
          distance = value - 10;
        passed_ms = 0;
      }
    }
};

extern ToF tof;
