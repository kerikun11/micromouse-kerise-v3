#pragma once

#include <Arduino.h>

#define PR_TX_SL_FR_PIN     16
#define PR_TX_SR_FL_PIN     17

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
        for (int k = 0; k < 2; k++) {
          for (int j = 0; j < ave_num; j++) raw[k][i][j] = 0;
          value[k][i] = 0;
        }
        offset[i] = 0;
      }
      pinMode(PR_TX_SL_FR_PIN, OUTPUT);
      pinMode(PR_TX_SR_FL_PIN, OUTPUT);
      timerSemaphore = xSemaphoreCreateBinary();
      xTaskCreatePinnedToCore([](void* obj) {
        static_cast<Reflector*>(obj)->task();
      }, "Reflector", REFLECTOR_TASK_STACK_SIZE, this, REFLECTOR_TASK_PRIORITY, &task_handle, 1);
    }
    enum REF_CH {
      REF_CH_SL,
      REF_CH_FL,
      REF_CH_FR,
      REF_CH_SR,
      REF_CH_MAX,
    };
    int16_t side(uint8_t left_or_right, const int range) const {
      if (left_or_right == 0) return read(REF_CH_SL, range);
      else return read(REF_CH_SR, range);;
    }
    int16_t front(uint8_t left_or_right, const int range) const {
      if (left_or_right == 0) return read(REF_CH_FL, range);
      else return read(REF_CH_FR, range);;
    }
    int16_t read(const enum REF_CH ch, const int range) const {
      if (ch < 0 || ch >= REF_CH_MAX || range < 0 || range >= 2) {
        log_e("you refered an invalid channel!");
        return 0;
      }
      return value[range][ch];
    }
    void csv() {
      printf("0,1800");
      for (int i = 0; i < REF_CH_MAX; i++) {
        for (int j = 0; j < 2; j++) {
          printf(",%d", value[j][i]);
        }
      }
      printf("\n");
    }
    void print() {
      printf("Reflector: ");
      for (int i = 0; i < REF_CH_MAX; i++) {
        for (int j = 0; j < 2; j++) {
          printf("\t%d", value[j][i]);
        }
      }
      printf("\n");
    }
    void give() {
      static portBASE_TYPE xHigherPriorityTaskWoken;
      xHigherPriorityTaskWoken = pdTRUE;
      xSemaphoreGiveFromISR(timerSemaphore, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR();
    }
  private:
    xTaskHandle task_handle;
    static const int ave_num = 4;
    int raw[2][REF_CH_MAX][ave_num];
    int value[2][REF_CH_MAX];
    int offset[REF_CH_MAX];
    const int rx_pins[REF_CH_MAX] = {PR_RX_SL_PIN, PR_RX_FL_PIN, PR_RX_FR_PIN, PR_RX_SR_PIN};
    const int tx_pins[REF_CH_MAX] = {PR_TX_SL_FR_PIN, PR_TX_SR_FL_PIN, PR_TX_SL_FR_PIN, PR_TX_SR_FL_PIN};
    volatile SemaphoreHandle_t timerSemaphore;

    void calibration() {
      printf("Reflector Offset: ");
      for (int i = 0; i < REF_CH_MAX; i++) {
        const int ave_count = 160;
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
        // 平均を計算するために過去 ave_num 個のデータを保持
        for (int k = 0; k < 2; k++) {
          for (int i = 0; i < REF_CH_MAX; i++) {
            for (int j = ave_num - 1; j > 0; j--) {
              raw[k][i][j] = raw[k][i][j - 1];
            }
          }
        }

        for (int i = 0; i < REF_CH_MAX; i++) {
          digitalWrite(tx_pins[i], LOW);
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); // 充電時間
          digitalWrite(tx_pins[i], HIGH);
          const int sample_wait_us = 10;
          delayMicroseconds(sample_wait_us);
          int temp = offset[i] - analogRead(rx_pins[i]);
          raw[0][i][0] = (temp < 0) ? 1 : temp;
        }
        //        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); // 放電時間

        for (int i = 0; i < REF_CH_MAX; i++) {
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); // 放電時間
          digitalWrite(tx_pins[i], LOW);
          delayMicroseconds(30); // 充電時間
          digitalWrite(tx_pins[i], HIGH);
          const int sample_wait_us = 10;
          delayMicroseconds(sample_wait_us);
          int temp = offset[i] - analogRead(rx_pins[i]);
          raw[1][i][0] = (temp < 0) ? 1 : temp;
          //          delayMicroseconds(30); // 放電時間
        }

        for (int k = 0; k < 2; k++) {
          for (int i = 0; i < REF_CH_MAX; i++) {
            int sum = 0;
            for (int j = 0; j < ave_num; j++) {
              sum += raw[k][i][j];
            }
            value[k][i] = sum / ave_num;
          }
        }
      }
    }
};

extern Reflector ref;

