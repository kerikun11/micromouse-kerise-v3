#pragma once

#include <Arduino.h>
#include "TaskBase.h"

#define PR_TX_SL_FR_PIN     16
#define PR_TX_SR_FL_PIN     17

#define PR_RX_SL_PIN        12
#define PR_RX_FL_PIN        13
#define PR_RX_FR_PIN        32
#define PR_RX_SR_PIN        33

#define REFLECTOR_TASK_PRIORITY   2
#define REFLECTOR_TASK_STACK_SIZE 512

class Reflector: private TaskBase {
  public:
    Reflector(): TaskBase("Reflector Task", REFLECTOR_TASK_PRIORITY, REFLECTOR_TASK_STACK_SIZE) {
    }
    int value[4];

    void init() {
      pinMode(PR_TX_SL_FR_PIN, OUTPUT);
      pinMode(PR_TX_SR_FL_PIN, OUTPUT);
      calibration();
      create_task();
    }
    void print() {
      printf("%d\t%d\t%d\t%d\n", value[0], value[1], value[2], value[3]);
    }
    void calibration() {
      printf("Reflector Offset: ");
      for (int i = 0; i < 4; i++) {
        const int ave_count = 100;
        offset[i] = 0;
        for (int t = 0; t < ave_count; t++) {
          offset[i] += analogRead(pins[i]);
          delay(1);
        }
        offset[i] /= ave_count;
        printf("%d\t", offset[i]);
      }
      printf("\n");
    }
  private:
    int offset[4];
    const int pins[4] = {PR_RX_SL_PIN, PR_RX_FL_PIN, PR_RX_FR_PIN, PR_RX_SR_PIN};

    void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      const int sample_wait_us = 10;
      const int charging_wait_us = 100;
      assert(adcAttachPin(PR_RX_SL_PIN));
      assert(adcAttachPin(PR_RX_FR_PIN));
      assert(adcAttachPin(PR_RX_FL_PIN));
      assert(adcAttachPin(PR_RX_SR_PIN));
      while (1) {
        digitalWrite(PR_TX_SL_FR_PIN, LOW);
        delayMicroseconds(charging_wait_us);
        digitalWrite(PR_TX_SL_FR_PIN, HIGH);
        delayMicroseconds(sample_wait_us);
        assert(adcStart(PR_RX_SL_PIN));
        assert(adcStart(PR_RX_FR_PIN));
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        value[0] = offset[0] - adcEnd(PR_RX_SL_PIN);
        value[2] = offset[2] - adcEnd(PR_RX_FR_PIN);

        digitalWrite(PR_TX_SR_FL_PIN, LOW);
        delayMicroseconds(charging_wait_us);
        digitalWrite(PR_TX_SR_FL_PIN, HIGH);
        delayMicroseconds(sample_wait_us);
        assert(adcStart(PR_RX_FL_PIN));
        assert(adcStart(PR_RX_SR_PIN));
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        value[1] = offset[1] - adcEnd(PR_RX_FL_PIN);
        value[3] = offset[3] - adcEnd(PR_RX_SR_PIN);
      }
    }
};

