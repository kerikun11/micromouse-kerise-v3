#pragma once

#include <Arduino.h>
#include <SPIFFS.h>
#include "config.h"

/* Hardware */
#include "reflector.h"
extern Reflector ref;
#include "tof.h"
extern ToF tof;

#define WALL_DETECTOR_TASK_PRIORITY   4
#define WALL_DETECTOR_STACK_SIZE      4096
#define WALL_UPDATE_PERIOD_US         1000

#define WALL_DETECTOR_BACKUP_PATH     "/WallDetector.bin"

#define WALL_DETECTOR_THRESHOLD_FRONT 120
#define WALL_DETECTOR_THRESHOLD_SIDE  60

class WallDetector {
  public:
    WallDetector() {}
    bool begin() {
      calibrationStartSemaphore = xSemaphoreCreateBinary();
      calibrationEndSemaphore = xSemaphoreCreateBinary();
      calibrationFrontStartSemaphore = xSemaphoreCreateBinary();
      calibrationFrontEndSemaphore = xSemaphoreCreateBinary();
      if (task_handle != NULL) {
        log_w("WallDetector is already running!");
      }
      xTaskCreate([](void* obj) {
        static_cast<WallDetector*>(obj)->task();
      }, "WallDetector", WALL_DETECTOR_STACK_SIZE, this, WALL_DETECTOR_TASK_PRIORITY, &task_handle);
      if (!restore()) return false;
      return true;
    }
    bool backup() {
      uint32_t us = micros();
      File file = SPIFFS.open(WALL_DETECTOR_BACKUP_PATH, FILE_WRITE);
      if (!file) {
        log_e("Can't open file!");
        return false;
      }
      file.write((const uint8_t*)(&(wall_ref)), sizeof(WallDetector::WallValue));
      log_d("Backup: %d [us]", micros() - us);
      return true;
    }
    bool restore() {
      File file = SPIFFS.open(WALL_DETECTOR_BACKUP_PATH, FILE_READ);
      if (!file) {
        log_e("Can't open file!");
        return false;
      }
      if (file.available() == sizeof(WallDetector::WallValue)) {
        uint8_t data[sizeof(WallDetector::WallValue)];
        file.read(data, sizeof(WallDetector::WallValue));
        memcpy((uint8_t*)(&(wall_ref)), data, sizeof(WallDetector::WallValue));
      } else {
        log_e("File size is invalid!");
        return false;
      }
      log_i("WallDetector Restore: %d %d %d %d",
            wall_ref.side[0], wall_ref.front[0], wall_ref.front[1], wall_ref.side[1]
           );
      return true;
    }
    void calibrationSide() {
      xSemaphoreTake(calibrationEndSemaphore, 0); //< 前のキャリブレーションの終了を待つ
      xSemaphoreGive(calibrationStartSemaphore);
      xSemaphoreTake(calibrationEndSemaphore, portMAX_DELAY);
    }
    void calibrationFront() {
      xSemaphoreTake(calibrationFrontEndSemaphore, 0); //< 前のキャリブレーションの終了を待つ
      xSemaphoreGive(calibrationFrontStartSemaphore);
      xSemaphoreTake(calibrationFrontEndSemaphore, portMAX_DELAY);
    }
    void print() {
      printf("Wall:\t%d\t%d\t%d\t%d\t[ %c %c %c ]\n",
             wall_diff.side[0],
             wall_diff.front[0],
             wall_diff.front[1],
             wall_diff.side[1],
             wall[0] ? 'X' : '.',
             wall[2] ? 'X' : '.',
             wall[1] ? 'X' : '.');
    }
    void csv() {
      printf("%d,%d,%d,%d\n",
             wall_diff.side[0],
             wall_diff.front[0],
             wall_diff.front[1],
             wall_diff.side[1]
            );
    }
    struct WallValue {
      int16_t side[2];
      int16_t front[2];
    };
    WallValue wall_ref;
    WallValue wall_diff;
    bool wall[3];
  private:
    xTaskHandle task_handle;
    SemaphoreHandle_t calibrationStartSemaphore;
    SemaphoreHandle_t calibrationEndSemaphore;
    SemaphoreHandle_t calibrationFrontStartSemaphore;
    SemaphoreHandle_t calibrationFrontEndSemaphore;
    static const int ave_num = 16;
    int16_t side_buf[ave_num][2];

    void calibration_side() {
      float sum[2] = {0.0f, 0.0f};
      const int ave_count = 500;
      for (int j = 0; j < ave_count; j++) {
        for (int i = 0; i < 2; i++) sum[i] += ref.side(i);
        delay(1);
      }
      for (int i = 0; i < 2; i++) wall_ref.side[i] = sum[i] / ave_count;
      printf("Wall Calibration Side:\t%04d\t%04d\n", (int) wall_ref.side[0], (int) wall_ref.side[1]);
    }
    void calibration_front() {
      float sum[2] = {0.0f, 0.0f};
      const int ave_count = 500;
      for (int j = 0; j < ave_count; j++) {
        for (int i = 0; i < 2; i++) sum[i] += ref.front(i);
        delay(1);
      }
      for (int i = 0; i < 2; i++) wall_ref.front[i] = sum[i] / ave_count;
      printf("Wall Calibration Front:\t%04d\t%04d\n", (int) wall_ref.front[0], (int) wall_ref.front[1]);
    }
    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);

        // detect front wall
        if (tof.getDistance() < WALL_DETECTOR_THRESHOLD_FRONT * 0.95f) wall[2] = true;
        else if (tof.getDistance() > WALL_DETECTOR_THRESHOLD_FRONT * 1.05f) wall[2] = false;
        if (tof.passedTimeMs() > 200) wall[2] = false;

        // detect side wall
        for (int i = 0; i < 2; i++) {
          for (int j = ave_num - 1; j > 0; j--) side_buf[j][i] = side_buf[j - 1][i];
          side_buf[0][i] = ref.side(i);
          int sum = 0;
          for (int j = 0; j < ave_num; j++) sum += side_buf[j][i];
          sum /= ave_num;
          if (sum > WALL_DETECTOR_THRESHOLD_SIDE * 1.1f) wall[i] = true;
          else if (sum < WALL_DETECTOR_THRESHOLD_SIDE * 0.9f) wall[i] = false;
        }

        for (int i = 0; i < 2; i++) {
          wall_diff.side[i] = ref.side(i) - wall_ref.side[i];
          wall_diff.front[i] = ref.front(i) - wall_ref.front[i];
        }
        if (xSemaphoreTake(calibrationStartSemaphore, 0) == pdTRUE) {
          calibration_side();
          xSemaphoreGive(calibrationEndSemaphore);
          xLastWakeTime = xTaskGetTickCount();
        }
        if (xSemaphoreTake(calibrationFrontStartSemaphore, 0) == pdTRUE) {
          calibration_front();
          xSemaphoreGive(calibrationFrontEndSemaphore);
          xLastWakeTime = xTaskGetTickCount();
        }
      }
    }
};

