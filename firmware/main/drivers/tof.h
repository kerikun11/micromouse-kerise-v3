#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "VL53L0X.h"

#define TOF_TASK_PRIORITY     1
#define TOF_TASK_STACK_SIZE   4096

class ToF {
  public:
    ToF(const int pin_sda, const int pin_scl): pin_sda(pin_sda), pin_scl(pin_scl) {}
    bool begin() {
      //      Wire.begin(pin_sda, pin_scl);
      i2c_config_t conf;
      conf.mode = I2C_MODE_MASTER;
      conf.sda_io_num = (gpio_num_t)pin_sda;
      conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
      conf.scl_io_num = (gpio_num_t)pin_scl;
      conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
      conf.master.clk_speed = 1000000;
      i2c_param_config(I2C_PORT_NUM_TOF, &conf);
      i2c_driver_install(I2C_PORT_NUM_TOF, conf.mode, 0, 0, 0);

      sensor.setTimeout(50);
      if (!sensor.init()) {
        log_e("ToF init failed :(");
        return false;
      }
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
      printf("0,90,180,270,360,%d\n", getDistance());
      //      printf("0,20,40,%d\n", passed_ms);
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
            vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); xLastWakeTime = xTaskGetTickCount();
            passed_ms++;
            if (millis() - startAt > 100) break;
          }
        }
        {
          uint32_t startAt = millis();
          while ((sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
            vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); xLastWakeTime = xTaskGetTickCount();
            passed_ms++;
            if (millis() - startAt > 100) break;
          }
        }
        uint16_t range = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
        sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
        if (range > 5 && range < 1000) {
          distance = range - 34;
          passed_ms = 0;
        }
      }
    }
};

