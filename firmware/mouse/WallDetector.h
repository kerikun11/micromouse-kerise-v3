#pragma once

#include <Arduino.h>
#include "config.h"

#include "reflector.h"
#include "tof.h"

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
    struct WALL_VALUE {
      int16_t side[2];
      int16_t front[2];
    };
    const WALL_VALUE& getRef() const {
      return wall_ref;
    }
    const WALL_VALUE& getDiff() const {
      return wall_diff;
    }
    bool getWall(const int ch) const {
      return wall[ch];
    }
  private:
    xTaskHandle task_handle;
    SemaphoreHandle_t calibrationStartSemaphore;
    SemaphoreHandle_t calibrationEndSemaphore;
    struct WALL_VALUE wall_ref;
    struct WALL_VALUE wall_diff;
    bool wall[3];

    void calibration_side() {
      float sum[2] = {0.0f, 0.0f};
      const int ave_count = 1000;
      for (int j = 0; j < ave_count; j++) {
        for (int i = 0; i < 2; i++) sum[i] += ref.side(i);
        delay(1);
      }
      for (int i = 0; i < 2; i++) wall_ref.side[i] = sum[i] / ave_count;
      for (int i = 0; i < 2; i++) wall_ref.front[i] =  WALL_DETECTOR_FLONT_RATIO * (wall_ref.side[0] + wall_ref.side[1]) / 2;
      printf("Wall Calibration:\t%04d\t%04d\t%04d\t%04d\n", (int) wall_ref.side[0], (int) wall_ref.front[0], (int) wall_ref.front[1], (int) wall_ref.side[1]);
    }
    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);

        const int threshold_front = 120;
        if (tof.getDistance() < threshold_front) wall[2] = true;
        else wall[2] = false;
        const int threshold_side = 75;
        for (int i = 0; i < 2; i++) {
          if (ref.side(i) > threshold_side) wall[i] = true;
          else wall[i] = false;
        }
        for (int i = 0; i < 2; i++) {
          wall_diff.side[i] = ref.side(i) - wall_ref.side[i];
          wall_diff.front[i] = ref.front(i) - wall_ref.front[i];
        }
        if (xSemaphoreTake(calibrationStartSemaphore, 0) == pdTRUE) {
          calibration_side();
          xSemaphoreGive(calibrationEndSemaphore);
        }
      }
    }
};

extern WallDetector wd;

