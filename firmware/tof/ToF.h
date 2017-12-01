#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "VL53L0X.h"

#define TOF_TASK_PRIORITY     0
#define TOF_TASK_STACK_SIZE   4096

class ToF {
  public:
    ToF(const int pin_sda, const int pin_scl): pin_sda(pin_sda), pin_scl(pin_scl) {}
    bool begin() {
      Wire.begin(pin_sda, pin_scl);
      Wire.reset();
      sensor.setTimeout(50);
      if (!sensor.init()) return false;
      //      sensor.setAddress(0x55);
      sensor.setMeasurementTimingBudget(20000);
      //      sensor.startContinuous();
      xTaskCreate([](void* obj) {
        static_cast<ToF*>(obj)->task();
      }, "ToF", TOF_TASK_STACK_SIZE, this, TOF_TASK_PRIORITY, NULL);
      return true;
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
      //      printf("0,90,180,270,360,%d\n", getDistance());
      printf("0,20,40,%d\n", passed_ms);
    }
  private:
    const int pin_sda, pin_scl;
    VL53L0X sensor;
    uint16_t distance;
    uint16_t passed_ms;

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
        {
          uint32_t startAt = millis();
          while (sensor.readReg(VL53L0X::SYSRANGE_START) & 0x01) {
            vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
            passed_ms++;
            if (millis() - startAt > 100) break;
          }
        }
        {
          uint32_t startAt = millis();
          while ((sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
            vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
            passed_ms++;
            if (millis() - startAt > 100) break;
          }
        }
        uint16_t range = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
        sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
        if (range > 5 && range < 300) {
          distance = range - 3;
          passed_ms = 0;
        }
      }
    }
};

extern ToF tof;
