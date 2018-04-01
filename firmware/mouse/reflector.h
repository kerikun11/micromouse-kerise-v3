#pragma once

#include <Arduino.h>
#include <array>
#include "TaskBase.h"

#define REFLECTOR_CH_SIZE     4

#define REFLECTOR_STACK_SIZE  4096
#define REFLECTOR_PRIORITY    10

class Reflector : TaskBase {
  public:
    Reflector(std::array<int8_t, REFLECTOR_CH_SIZE> tx_pins, std::array<int8_t, REFLECTOR_CH_SIZE> rx_pins): tx_pins(tx_pins), rx_pins(rx_pins) {}
    bool begin() {
      for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) {
        value[i] = 0;
        offset[i] = 0;
        pinMode(tx_pins[i], OUTPUT);
        digitalWrite(tx_pins[i], LOW);
      }
      return createTask("Reflector", REFLECTOR_PRIORITY, REFLECTOR_STACK_SIZE, 1);
    }
    int16_t side(uint8_t isRight) const {
      if (isRight == 0) return read(0);
      else return read(3);;
    }
    int16_t front(uint8_t isRight) const {
      if (isRight == 0) return read(1);
      else return read(2);
    }
    int16_t read(const int8_t ch) const {
      if (ch < 0 || ch >= REFLECTOR_CH_SIZE) {
        log_e("you refered an invalid channel!");
        return 0;
      }
      return value[ch];
    }
    void csv() const {
      printf("0,1500");
      for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) printf(",%d", value[i]);
      printf("\n");
    }
    void print() const {
      printf("Reflector: ");
      for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) printf("\t%04d", value[i]);
      printf("\n");
    }
  private:
    static const int ave_num = 2;
    const std::array<int8_t, REFLECTOR_CH_SIZE> tx_pins;
    const std::array<int8_t, REFLECTOR_CH_SIZE> rx_pins;
    int16_t value_buffer[ave_num][REFLECTOR_CH_SIZE];
    int16_t value[REFLECTOR_CH_SIZE];
    int16_t offset[REFLECTOR_CH_SIZE];

    void calibration() {
      for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) {
        const int ave_count = 100;
        int sum = 0;
        portTickType xLastWakeTime = xTaskGetTickCount();
        for (int t = 0; t < ave_count; t++) {
          sum += analogRead(rx_pins[i]);
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        }
        offset[i] = sum / ave_count;
      }
      log_d("Reflector Offset:\t%d\t%d\t%d\t%d", offset[0], offset[1], offset[2], offset[3]);
    }
    void task() {
      calibration();

      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        // Buffer shift
        for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) {
          for (int j = ave_num - 1; j > 0; j--) {
            value_buffer[j][i] = value_buffer[j - 1][i];
          }
        }
        // Sampling
        for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) {
          digitalWrite(tx_pins[i], LOW);        //< 充電開始
          delayMicroseconds(50);                //< 充電時間
          digitalWrite(tx_pins[i], HIGH);       //< 放電開始
          delayMicroseconds(15);                //< 最大振幅になるまでの待ち時間
          int raw = analogRead(rx_pins[i]);     //< サンプリング
          int temp = offset[i] - raw;           //< オフセットとの差をとる
          value_buffer[0][i] = std::max(temp, 1);//< 0以下にならないように1で飽和
          delayMicroseconds(50);                 // 放電時間
        }
        // LPF
        for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) {
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

