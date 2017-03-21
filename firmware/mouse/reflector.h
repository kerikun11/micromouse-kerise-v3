#pragma once

#include <Arduino.h>
#include "TaskBase.h"

#define PR_TX_SL_FR_PIN     16
#define PR_TX_SR_FL_PIN     17

#define PR_RX_SL_PIN        12
#define PR_RX_FL_PIN        13
#define PR_RX_FR_PIN        32
#define PR_RX_SR_PIN        33

#define REFLECTOR_TASK_PRIORITY   5
#define REFLECTOR_TASK_STACK_SIZE 1024

class Reflector: private TaskBase {
  public:
    Reflector(): TaskBase("Reflector Task", REFLECTOR_TASK_PRIORITY, REFLECTOR_TASK_STACK_SIZE) {}
    virtual ~Reflector() {}
    void enable() {
      //      delete_task();
      pinMode(PR_TX_SL_FR_PIN, OUTPUT);
      pinMode(PR_TX_SR_FL_PIN, OUTPUT);
      assert(adcAttachPin(PR_RX_SL_PIN));
      assert(adcAttachPin(PR_RX_FR_PIN));
      assert(adcAttachPin(PR_RX_FL_PIN));
      assert(adcAttachPin(PR_RX_SR_PIN));
      calibration();
      create_task(1);
    }
    void disable() {
      delete_task();
      digitalWrite(PR_TX_SL_FR_PIN, LOW);
      digitalWrite(PR_TX_SL_FR_PIN, LOW);
    }
    int16_t side(uint8_t left_or_right) {
      if (left_or_right == 0)
        return value[0];
      else
        return value[3];
    }
    int16_t front(uint8_t left_or_right) {
      if (left_or_right == 0)
        return value[1];
      else
        return value[2];
    }
    void print() {
      printf("%d\t%d\t%d\t%d\n", value[0], value[1], value[2], value[3]);
    }
  private:
    int value[4];
    int offset[4];
    const int pins[4] = {PR_RX_SL_PIN, PR_RX_FL_PIN, PR_RX_FR_PIN, PR_RX_SR_PIN};

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
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      const int sample_wait_us = 10;
      const int charging_wait_us = 100;
      int temp;
      while (1) {
        digitalWrite(PR_TX_SL_FR_PIN, LOW);
        delayMicroseconds(charging_wait_us);
        digitalWrite(PR_TX_SL_FR_PIN, HIGH);
        delayMicroseconds(sample_wait_us);
        assert(adcStart(PR_RX_FR_PIN));
        assert(adcStart(PR_RX_SL_PIN));
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        temp = offset[0] - adcEnd(PR_RX_SL_PIN);
        value[0] = (temp < 0) ? 1 : temp;
        temp = offset[2] - adcEnd(PR_RX_FR_PIN);
        value[2] = (temp < 0) ? 1 : temp;

        digitalWrite(PR_TX_SR_FL_PIN, LOW);
        delayMicroseconds(charging_wait_us);
        digitalWrite(PR_TX_SR_FL_PIN, HIGH);
        delayMicroseconds(sample_wait_us);
        assert(adcStart(PR_RX_SR_PIN));
        assert(adcStart(PR_RX_FL_PIN));
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        temp = offset[1] - adcEnd(PR_RX_FL_PIN);
        value[1] = (temp < 0) ? 1 : temp;
        temp = offset[3] - adcEnd(PR_RX_SR_PIN);
        value[3] = (temp < 0) ? 1 : temp;
      }
    }
};

