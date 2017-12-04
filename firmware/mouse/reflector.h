#pragma once

#include <Arduino.h>
#include <algorithm>

#define REFLECTOR_TASK_PRIORITY   10
#define REFLECTOR_TASK_STACK_SIZE 4096

#define REFLECTOR_CH_SIZE         4

class Reflector {
  public:
    Reflector(std::array<int, REFLECTOR_CH_SIZE> tx_pins, std::array<int, REFLECTOR_CH_SIZE> rx_pins): tx_pins(tx_pins), rx_pins(rx_pins) {}
    void begin() {
      for (int i = 0; i < REFLECTOR_CH_SIZE; i++) {
        value[i] = 0;
        offset[i] = 0;
        pinMode(tx_pins[i], OUTPUT);
      }
      analogSetCycles(2);
      if (task_handle == NULL) {
        xTaskCreatePinnedToCore([](void* obj) {
          static_cast<Reflector*>(obj)->task();
        }, "Reflector", REFLECTOR_TASK_STACK_SIZE, this, REFLECTOR_TASK_PRIORITY, &task_handle, 1);
      }
    }
    int16_t side(uint8_t isRight) const {
      if (isRight == 0) return read(0);
      else return read(3);;
    }
    int16_t front(uint8_t isRight) const {
      if (isRight == 0) return read(1);
      else return read(2);
    }
    int16_t read(const int ch) const {
      if (ch < 0 || ch >= REFLECTOR_CH_SIZE) {
        log_e("you refered an invalid channel!");
        return 0;
      }
      return value[ch];
    }
    void csv() const {
      printf("0,900");
      for (int i = 0; i < REFLECTOR_CH_SIZE; i++) printf(",%d", value[i]);
      printf("\n");
    }
    void print() const {
      printf("Reflector: ");
      for (int i = 0; i < REFLECTOR_CH_SIZE; i++) printf("\t%04d", value[i]);
      printf("\n");
    }
  private:
    xTaskHandle task_handle;
    static const int ave_num = 2;
    int16_t value_buffer[ave_num][REFLECTOR_CH_SIZE];
    const std::array<int, REFLECTOR_CH_SIZE> tx_pins;
    const std::array<int, REFLECTOR_CH_SIZE> rx_pins;
    int16_t value[REFLECTOR_CH_SIZE];
    int16_t offset[REFLECTOR_CH_SIZE];

    void calibration() {
      printf("Reflector Offset: ");
      for (int i = 0; i < REFLECTOR_CH_SIZE; i++) {
        const int ave_count = 100;
        int sum = 0;
        for (int t = 0; t < ave_count; t++) {
          sum += analogRead(rx_pins[i]);
          delay(1);
        }
        offset[i] = sum / ave_count;
        printf("%d\t", offset[i]);
      }
      printf("\n");
    }
    void task() {
      calibration();

      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); //< 同期
        // Buffer shift
        for (int i = 0; i < REFLECTOR_CH_SIZE; i++) {
          for (int j = ave_num - 1; j > 0; j--) {
            value_buffer[j][i] = value_buffer[j - 1][i];
          }
        }
        // Sampling
        for (int i = 0; i < REFLECTOR_CH_SIZE; i++) {
          portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
          digitalWrite(tx_pins[i], LOW);    //< 充電開始
          delayMicroseconds(50);            //< 充電時間
          taskENTER_CRITICAL(&myMutex);
          digitalWrite(tx_pins[i], HIGH);   //< 放電開始
          delayMicroseconds(15);             //< 最大振幅になるまでの待ち時間
          int raw = analogRead(rx_pins[i]); //< サンプリング
          taskEXIT_CRITICAL(&myMutex);
          int temp = offset[i] - raw;       //< オフセットとの差をとる
          value_buffer[0][i] = (temp < 0) ? 1 : temp; //< 0以下にならないように飽和
          delayMicroseconds(50);           // 放電時間
        }
        // LPF
        for (int i = 0; i < REFLECTOR_CH_SIZE; i++) {
          int sum = 0;
          for (int j = 0; j < ave_num; j++) {
            sum += value_buffer[j][i];
          }
          value[i] = sum / ave_num;
        }
      }
    }
};

extern Reflector ref;

