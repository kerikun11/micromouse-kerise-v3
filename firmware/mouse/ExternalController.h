#pragma once

#include <Arduino.h>
#include "config.h"

#include "Buzzer.h"
extern Buzzer bz;
#include "imu.h"
extern IMU imu;

#define EXTERNAL_CONTROLLER_TASK_PRIORITY 1
#define EXTERNAL_CONTROLLER_STACK_SIZE    4096

class ExternalController {
  public:
    ExternalController() {}
    void begin() {
      xTaskCreate([](void* obj) {
        static_cast<ExternalController*>(obj)->task();
      }, "ExternalController", EXTERNAL_CONTROLLER_STACK_SIZE, this, EXTERNAL_CONTROLLER_TASK_PRIORITY, NULL);
    }
  private:
    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); xLastWakeTime = xTaskGetTickCount();
        while (Serial.available()) {
          char c = Serial.read();
          printf("%c\n", c);
          switch (c) {
            case 't':
              bz.play(Buzzer::CONFIRM);
              imu.calibration();
              break;
          }
        }
      }
    }
};

