#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"

#include "reflector.h"

#define WALL_DETECTOR_TASK_PRIORITY 4
#define WALL_DETECTOR_STACK_SIZE    4096

#define WALL_DETECTOR_FLONT_RATIO   1.0f
#define WALL_SIDE_DIV               2.4f  //< Response
#define WALL_FRONT_DIV              2.6f  //< Response

#define WALL_UPDATE_PERIOD_US       1000

class WallDetector : TaskBase {
  public:
    WallDetector() : TaskBase("Wall Detector", WALL_DETECTOR_TASK_PRIORITY, WALL_DETECTOR_STACK_SIZE), calibration_flag(false) {}
    virtual ~WallDetector() {}
    void enable() {
      create_task();
    }
    void disable() {
      delete_task();
    }
    void calibration(bool waitUntilTheEnd = true) {
      calibration_flag = true;
      if (waitUntilTheEnd) {
        calibrationWait();
      }
    }
    void calibrationWait() {
      while (calibration_flag) {
        vTaskDelay(1 / portTICK_RATE_MS);
      }
    }
    void print() {
      printf("Wall:\t%04u\t%04u\t%04u\t%04u\t|\t", ref.side(0), ref.front(0), ref.front(1), ref.side(1));
      printf("%05.3f\t%05.3f\t%05.3f\t%05.3f\t|\t", wall_difference().side[0], wall_difference().front[0],
             wall_difference().front[1], wall_difference().side[1]);
      printf("%s %s %s %s\n", wall().side[0] ? "X" : ".", wall().front[0] ? "X" : ".",
             wall().front[1] ? "X" : ".", wall().side[1] ? "X" : ".");
    }
    struct WALL {
      bool side[2];
      bool front[2];
    };
    struct WALL_VALUE {
      float side[2];
      float front[2];
    };
    struct WALL wall() {
      return _wall;
    }
    struct WALL_VALUE wall_distance() {
      return _wall_distance;
    }
    struct WALL_VALUE wall_difference() {
      return _wall_difference;
    }
  private:
    bool calibration_flag;
    struct WALL _wall;
    struct WALL_VALUE _wall_ref;
    struct WALL_VALUE _wall_distance;
    struct WALL_VALUE _wall_difference;
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);

        for (int i = 0; i < 2; i++) {
          int16_t value = ref.side(i);
          if (value > _wall_ref.side[i] * 1.02)
            _wall.side[i] = true;
          else if (value < _wall_ref.side[i] * 0.98)
            _wall.side[i] = false;
          _wall_difference.side[i] = (_wall_distance.side[i] - value) / _wall_distance.side[i];
        }
        for (int i = 0; i < 2; i++) {
          int16_t value = (ref.front(0) + ref.front(1)) / 2;
          if (value > _wall_ref.front[i] * 1.02)
            _wall.front[i] = true;
          else if (value < _wall_ref.front[i] * 0.98)
            _wall.front[i] = false;
          //          _wall_difference.front[i] = (_wall_distance.front[i] - value) / _wall_distance.front[i];
        }
        for (int i = 0; i < 2; i++) {
          int16_t value = ref.front(i);
          //          if (value > _wall_ref.front[i] * 1.02)
          //            _wall.front[i] = true;
          //          else if (value < _wall_ref.front[i] * 0.98)
          //            _wall.front[i] = false;
          _wall_difference.front[i] = (_wall_distance.front[i] - value) / _wall_distance.front[i];
        }

        if (calibration_flag) {
          static float sum[2] = {0, 0};
          for (int i = 0; i < 2; i++) {
            sum[i] += ref.side(i);
          }

          static int calibration_counter = 0;
          const int ave_count = 1000;
          if (++calibration_counter >= ave_count) {
            for (int i = 0; i < 2; i++) {
              _wall_distance.side[i] = sum[i] / ave_count;
              _wall_ref.side[i] = _wall_distance.side[i] / WALL_SIDE_DIV;
            }
            for (int i = 0; i < 2; i++) {
              _wall_distance.front[i] = WALL_DETECTOR_FLONT_RATIO * (_wall_distance.side[0] + _wall_distance.side[1]) / 2;
              _wall_ref.front[i] = _wall_distance.front[i] / WALL_FRONT_DIV;
            }
            printf("Wall Calibration:\t%04d\t%04d\t%04d\t%04d\n", (int) _wall_distance.side[0],
                   (int) _wall_distance.front[0], (int) _wall_distance.front[1], (int) _wall_distance.side[1]);
            for (int i = 0; i < 2; i++) {
              sum[i] = 0;
            }
            calibration_counter = 0;
            calibration_flag = false;
          }
        }
      }
    }
};

extern WallDetector wd;

