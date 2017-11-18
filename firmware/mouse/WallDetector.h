#pragma once

#include <Arduino.h>
#include "config.h"

#include "reflector.h"
#include "tof.h"

#define WALL_DETECTOR_TASK_PRIORITY 4
#define WALL_DETECTOR_STACK_SIZE    4096

#define WALL_DETECTOR_FLONT_RATIO   2.6f

#define WALL_UPDATE_PERIOD_US       1000

class WallDetector {
  public:
    WallDetector() {}
    void begin() {
      calibrationStartSemaphore = xSemaphoreCreateBinary();
      calibrationEndSemaphore = xSemaphoreCreateBinary();
      calibrationFrontStartSemaphore = xSemaphoreCreateBinary();
      calibrationFrontEndSemaphore = xSemaphoreCreateBinary();
      if (task_handle == NULL) {
        xTaskCreate([](void* obj) {
          static_cast<WallDetector*>(obj)->task();
        }, "WallDetector", WALL_DETECTOR_STACK_SIZE, this, WALL_DETECTOR_TASK_PRIORITY, &task_handle);
      }
    }
    //    void calibration(bool waitUntilTheEnd = true) {
    //      xSemaphoreTake(calibrationEndSemaphore, 0); //< 前のキャリブレーションの終了を待つ
    //      xSemaphoreGive(calibrationStartSemaphore);
    //      if (waitUntilTheEnd) calibrationWait();
    //    }
    //    void calibrationWait() {
    //      xSemaphoreTake(calibrationEndSemaphore, portMAX_DELAY);
    //    }
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
      //      for (int i = 0; i < 2; i++) wall_ref.front[i] =  WALL_DETECTOR_FLONT_RATIO * (wall_ref.side[0] + wall_ref.side[1]) / 2;
      //      wall_ref.front[0] += 4;
      //      wall_ref.front[1] -= 4;
      //      printf("Wall Calibration Side:\t%04d\t%04d\t%04d\t%04d\n", (int) wall_ref.side[0], (int) wall_ref.front[0], (int) wall_ref.front[1], (int) wall_ref.side[1]);
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

        const int threshold_front = 120;
        if (tof.getDistance() < threshold_front * 0.95f) wall[2] = true;
        else if (tof.getDistance() > threshold_front * 1.05f) wall[2] = false;

        const int threshold_side = 60;
        for (int i = 0; i < 2; i++) {
          for (int j = ave_num - 1; j > 0; j--) side_buf[j][i] = side_buf[j - 1][i];
          side_buf[0][i] = ref.side(i);
          int sum = 0;
          for (int j = 0; j < ave_num; j++) sum += side_buf[j][i];
          sum /= ave_num;
          if (sum > threshold_side * 1.1f) wall[i] = true;
          else if (sum < threshold_side * 0.9f) wall[i] = false;
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

extern WallDetector wd;

