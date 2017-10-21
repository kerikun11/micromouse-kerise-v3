#pragma once

#include <Arduino.h>

#define PR_TX_SL_PIN     16
#define PR_TX_FL_PIN     17
#define PR_TX_FR_PIN     16
#define PR_TX_SR_PIN     17

#define PR_RX_SL_PIN        12
#define PR_RX_FL_PIN        13
#define PR_RX_FR_PIN        32
#define PR_RX_SR_PIN        33

#define REFLECTOR_TASK_PRIORITY   6
#define REFLECTOR_TASK_STACK_SIZE 4096

class Reflector {
  public:
    Reflector() {}
    void begin() {
      for (int i = 0; i < REF_CH_MAX; i++) {
        for (int j = 0; j < ave_num; j++) raw[i][j] = 0;
        value[i] = 0;
        offset[i] = 0;
        pinMode(tx_pins[i], OUTPUT);
      }
      //      timerSemaphore = xSemaphoreCreateBinary();
      oneshotStartSemaphore = xSemaphoreCreateBinary();
      oneshotEndSemaphore = xSemaphoreCreateBinary();
      if (task_handle == NULL) {
        xTaskCreatePinnedToCore([](void* obj) {
          static_cast<Reflector*>(obj)->task();
        }, "Reflector", REFLECTOR_TASK_STACK_SIZE, this, REFLECTOR_TASK_PRIORITY, &task_handle, 1);
      }
    }
    enum REF_CH {
      REF_CH_SL,
      REF_CH_FL,
      REF_CH_FR,
      REF_CH_SR,
      REF_CH_MAX,
    };
    int16_t side(uint8_t left_or_right) const {
      if (left_or_right == 0) return read(REF_CH_SL);
      else return read(REF_CH_SR);;
    }
    int16_t front(uint8_t left_or_right) const {
      if (left_or_right == 0) return read(REF_CH_FL);
      else return read(REF_CH_FR);;
    }
    int16_t read(const enum REF_CH ch) const {
      if (ch < 0 || ch >= REF_CH_MAX) {
        log_e("you refered an invalid channel!");
        return 0;
      }
      return value[ch];
    }
    void csv() {
      printf("0,1800");
      for (int i = 0; i < REF_CH_MAX; i++) {
        printf(",%d", value[i]);
      }
      for (int i = 0; i < REF_CH_MAX; i++) {
        printf(",%d", value_oneshot[i]);
      }
      printf("\n");
    }
    void print() {
      printf("Reflector: ");
      for (int i = 0; i < REF_CH_MAX; i++) {
        printf("\t%d", value[i]);
      }
      printf("\n");
    }
    void printOneshot() {
      printf("Reflector: ");
      for (int i = 0; i < REF_CH_MAX; i++) {
        printf("\t%d", value_oneshot[i]);
      }
      printf("\n");
    }
    //    void give() {
    //      static portBASE_TYPE xHigherPriorityTaskWoken;
    //      xHigherPriorityTaskWoken = pdTRUE;
    //      xSemaphoreGiveFromISR(timerSemaphore, &xHigherPriorityTaskWoken);
    //      portYIELD_FROM_ISR();
    //    }
    void oneshot() {
      xSemaphoreGive(oneshotStartSemaphore);
      xSemaphoreTake(oneshotEndSemaphore, portMAX_DELAY);
    }
    int getOneshotValue(const enum REF_CH ch) const {
      if (ch < 0 || ch >= REF_CH_MAX) {
        log_e("you refered an invalid channel!");
        return 0;
      }
      return value_oneshot[ch];
    }
  private:
    xTaskHandle task_handle;
    static const int ave_num = 2;
    int raw[REF_CH_MAX][ave_num];
    int value[REF_CH_MAX];
    int value_oneshot[REF_CH_MAX];
    int offset[REF_CH_MAX];
    const int rx_pins[REF_CH_MAX] = {PR_RX_SL_PIN, PR_RX_FL_PIN, PR_RX_FR_PIN, PR_RX_SR_PIN};
    const int tx_pins[REF_CH_MAX] = {PR_TX_SL_PIN, PR_TX_FL_PIN, PR_TX_FR_PIN, PR_TX_SR_PIN};
    //    volatile SemaphoreHandle_t timerSemaphore;
    SemaphoreHandle_t oneshotStartSemaphore;
    SemaphoreHandle_t oneshotEndSemaphore;

    void calibration() {
      printf("Reflector Offset: ");
      for (int i = 0; i < REF_CH_MAX; i++) {
        const int ave_count = 100;
        offset[i] = 0;
        for (int t = 0; t < ave_count; t++) {
          offset[i] += analogRead(rx_pins[i]);
          delay(1);
        }
        offset[i] /= ave_count;
        printf("%d\t", offset[i]);
      }
      printf("\n");
    }
    void task() {
      calibration();

      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); //< 同期
        // Sampling
        for (int i = 0; i < REF_CH_MAX; i++) {
          digitalWrite(tx_pins[i], LOW);
          delayMicroseconds(30); // 充電時間
          digitalWrite(tx_pins[i], HIGH);
          const int sample_wait_us = 10;
          delayMicroseconds(sample_wait_us);
          int temp = offset[i] - analogRead(rx_pins[i]);
          raw[i][0] = (temp < 0) ? 1 : temp;
        }
        // 平均を計算
        for (int i = 0; i < REF_CH_MAX; i++) {
          int sum = 0;
          for (int j = 0; j < ave_num; j++) sum += raw[i][j];
          value[i] = sum / ave_num;
          for (int j = ave_num - 1; j > 0; j--) raw[i][j] = raw[i][j - 1]; //< ずらす
        }
        // oneshotが要求されていれば実行
        if (xSemaphoreTake(oneshotStartSemaphore, 0) == pdTRUE) {
          for (int i = 0; i < REF_CH_MAX; i++) {
            digitalWrite(tx_pins[i], LOW);
            vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); // 充電時間
            digitalWrite(tx_pins[i], HIGH);
            const int sample_wait_us = 10;
            delayMicroseconds(sample_wait_us); // 波形がピークになるまで待つ
            int temp = offset[i] - analogRead(rx_pins[i]);
            value_oneshot[i] = (temp < 0) ? 1 : temp;
          }
          xSemaphoreGive(oneshotEndSemaphore);
        }
      }
    }
};

extern Reflector ref;

