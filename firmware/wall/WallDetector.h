#pragma once

#include <Arduino.h>
#include "config.h"

#include "reflector.h"

#define WALL_DETECTOR_TASK_PRIORITY 4
#define WALL_DETECTOR_STACK_SIZE    4096

#define WALL_DETECTOR_FLONT_RATIO   2.4f

#define WALL_UPDATE_PERIOD_US       1000

class WallDetector {
  public:
    WallDetector() {}
    void begin() {
      calibrationStartSemaphore = xSemaphoreCreateBinary();
      calibrationEndSemaphore = xSemaphoreCreateBinary();
      if (task_handle == NULL) {
        xTaskCreate([](void* obj) {
          static_cast<WallDetector*>(obj)->task();
        }, "WallDetector", WALL_DETECTOR_STACK_SIZE, this, WALL_DETECTOR_TASK_PRIORITY, &task_handle);
      }
    }
    void calibration(bool waitUntilTheEnd = true) {
      xSemaphoreTake(calibrationEndSemaphore, 0); //< 前のキャリブレーションの終了を待つ
      xSemaphoreGive(calibrationStartSemaphore);
      if (waitUntilTheEnd) calibrationWait();
    }
    void calibrationWait() {
      xSemaphoreTake(calibrationEndSemaphore, portMAX_DELAY);
    }
    void print() {
      printf("Wall:\t%05.3f\t%05.3f\t%05.3f\t%05.3f\t|\n", wall_ratio().side[0], wall_ratio().front[0], wall_ratio().front[1], wall_ratio().side[1]);
    }
    struct WALL_VALUE {
      float side[2];
      float front[2];
    };
    struct WALL_VALUE wall_distance() {
      return _wall_distance;
    }
    struct WALL_VALUE wall_ratio() {
      return _wall_ratio;
    }
    uint8_t wallDetect() {
      uint8_t wall = 0;
      const int threshold[4] = {500, 270, 270, 500};
      ref.oneshot();
      for (int i = 0; i < 4; i++) {
        if (ref.getOneshotValue(static_cast<enum Reflector::REF_CH>(i)) > threshold[i]) wall |= 1 << i;
      }
      printf("Wall: ");
      for (int i = 0; i < 4; i++) printf("%s ", ((wall >> i) & 1) ? "X" : ".");
      printf("\n");
      return wall;
    }
  private:
    xTaskHandle task_handle;
    SemaphoreHandle_t calibrationStartSemaphore;
    SemaphoreHandle_t calibrationEndSemaphore;
    struct WALL_VALUE _wall_distance;
    struct WALL_VALUE _wall_ratio;
    void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);

        for (int i = 0; i < 2; i++) {
          int16_t value = ref.side(i);
          _wall_ratio.side[i] = (_wall_distance.side[i] - value) / _wall_distance.side[i];
        }
        for (int i = 0; i < 2; i++) {
          int16_t value = ref.front(i);
          _wall_ratio.front[i] =  (_wall_distance.front[i] * WALL_DETECTOR_FLONT_RATIO - value) / _wall_distance.front[i];
        }

        if (xSemaphoreTake(calibrationStartSemaphore, 0) == pdTRUE) {
          float sum[2] = {0.0f, 0.0f};
          const int ave_count = 1000;
          for (int i = 0; i < 2; i++) for (int j = 0; j < ave_count; j++) sum[i] += ref.side(i);
          for (int i = 0; i < 2; i++) _wall_distance.side[i] = sum[i] / ave_count;
          for (int i = 0; i < 2; i++) _wall_distance.front[i] =  (_wall_distance.side[0] + _wall_distance.side[1]) / 2;
          printf("Wall Calibration:\t%04d\t%04d\t%04d\t%04d\n", (int) _wall_distance.side[0],
                 (int) _wall_distance.front[0], (int) _wall_distance.front[1], (int) _wall_distance.side[1]);
          xSemaphoreGive(calibrationEndSemaphore);
        }
      }
    }
};

extern WallDetector wd;

