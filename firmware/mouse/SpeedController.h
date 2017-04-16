#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "config.h"

#include "as5145.h"
#include "motor.h"
#include "mpu6500.h"

class Position {
  public:
    Position(const float x = 0, const float y = 0, const float theta = 0) :
      x(x), y(y), theta(theta) {
    }
    Position(const float pos[3]) :
      x(pos[0]), y(pos[1]), theta(pos[2]) {
    }
    float x, y, theta;

    inline void reset() {
      x = 0;
      y = 0;
      theta = 0;
    }
    inline void set(const float x = 0, const float y = 0, const float theta = 0) {
      this->x = x;
      this->y = y;
      this->theta = theta;
    }
    inline Position rotate(const float angle) {
      float _x = x;
      float _y = y;
      x = _x * cos(angle) - _y * sin(angle);
      y = _x * sin(angle) + _y * cos(angle);
      return *this;
    }
    inline float getNorm() const {
      return sqrt(x * x + y * y);
    }
    inline Position mirror_x() const {
      return Position(x, -y, -theta);
    }
    inline Position operator=(const Position &obj) {
      x = obj.x;
      y = obj.y;
      theta = obj.theta;
      return *this;
    }
    inline Position operator+() const {
      return Position(x, y, theta);
    }
    inline Position operator+(const Position &obj) const {
      return Position(x + obj.x, y + obj.y, theta + obj.theta);
    }
    inline Position operator+=(const Position &obj) {
      x += obj.x;
      y += obj.y;
      theta += obj.theta;
      return *this;
    }
    inline Position operator-() const {
      return Position(-x, -y, -theta);
    }
    inline Position operator-(const Position &obj) const {
      return Position(x - obj.x, y - obj.y, theta - obj.theta);
    }
    inline Position operator-=(const Position &obj) {
      x -= obj.x;
      y -= obj.y;
      theta -= obj.theta;
      return *this;
    }
    inline Position operator/(const float &div) {
      return Position(x / div, y / div, theta);
    }
    inline Position operator/=(const float &div) {
      x /= div;
      y /= div;
      return *this;
    }
    inline Position operator*(const float &div) {
      return Position(x * div, y * div, theta);
    }
    inline Position operator*=(const float &div) {
      x *= div;
      y *= div;
      return *this;
    }
    inline void print(const char* name = "") {
      printf("%s: (%.1f,\t%.1f,\t%.3f)\n", name, x, y, theta);
    }
};

#define SPEED_CONTROLLER_TASK_PRIORITY  4
#define SPEED_CONTROLLER_STACK_SIZE     4096

#define SPEED_CONTROLLER_KP   1.6f
#define SPEED_CONTROLLER_KI   3.0f
#define SPEED_CONTROLLER_KD   0.004f

#define SPEED_CONTROLLER_KP_SUCTION 3.6f
#define SPEED_CONTROLLER_KI_SUCTION 2.1f
#define SPEED_CONTROLLER_KD_SUCTION 0.0018f

#define SPEED_CONTROLLER_PERIOD_US  1000

class SpeedController : TaskBase {
  public:
    SpeedController() : TaskBase("Speed Controller", SPEED_CONTROLLER_TASK_PRIORITY, SPEED_CONTROLLER_STACK_SIZE) {
      reset();
    }
    virtual ~SpeedController() {}
    struct WheelParameter {
      WheelParameter(float trans = 0, float rot = 0) :
        trans(trans), rot(rot) {
      }
      float trans;  //< translation
      float rot;    //< rotation
      float wheel[2]; //< wheel [0]: left, [1]: right
      void pole2wheel() {
        wheel[0] = trans - MACHINE_ROTATION_RADIUS * rot;
        wheel[1] = trans + MACHINE_ROTATION_RADIUS * rot;
      }
      void wheel2pole() {
        rot = (wheel[1] - wheel[0]) / 2.0f / MACHINE_ROTATION_RADIUS;
        trans = (wheel[1] + wheel[0]) / 2.0f;
      }
    };
    void reset() {
      for (int i = 0; i < 2; i++) {
        target.wheel[i] = 0;
        for (int j = 0; j < ave_num; j++) {
          wheel_position[j][i] = as.position(i);
          accel[j] = 0;
        }
        actual.wheel[i] = 0;
        integral.wheel[i] = 0;
        differential.wheel[i] = 0;
      }
      actual_prev.trans = 0;
      actual_prev.rot = 0;
      position.reset();
    }
    void enable(bool suction = false) {
      if (suction) {
        Kp = SPEED_CONTROLLER_KP_SUCTION;
        Ki = SPEED_CONTROLLER_KI_SUCTION;
        Kd = SPEED_CONTROLLER_KD_SUCTION;
      } else {
        Kp = SPEED_CONTROLLER_KP;
        Ki = SPEED_CONTROLLER_KI;
        Kd = SPEED_CONTROLLER_KD;
      }
      reset();
      create_task();
      printf("Speed Controller Enabled\n");
    }
    void disable() {
      delete_task();
      mt.free();
    }
    void set_target(float trans, float rot) {
      target.trans = trans;
      target.rot = rot;
      target.pole2wheel();
    }
    Position& getPosition() {
      return position;
    }
    //    WheelParameter real;  /* debug */
    WheelParameter target;
    WheelParameter actual;
    WheelParameter integral;
    WheelParameter differential;
    float Kp = SPEED_CONTROLLER_KP;
    float Ki = SPEED_CONTROLLER_KI;
    float Kd = SPEED_CONTROLLER_KD;
  private:
    static const int ave_num = 16;
    float wheel_position[ave_num][2];
    float accel[ave_num];
    WheelParameter actual_prev;
    WheelParameter target_prev;
    Position position;

    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        for (int i = 0; i < 2; i++) {
          for (int j = ave_num - 1; j > 0; j--) {
            wheel_position[j][i] = wheel_position[j - 1][i];
          }
          wheel_position[0][i] = as.position(i);
        }
        for (int j = ave_num - 1; j > 0; j--) {
          accel[j] = accel[j - 1];
        }
        accel[0] = mpu.accel.y;
        float sum_accel = 0.0f;
        for (int j = 0; j < ave_num; j++) {
          sum_accel += accel[j];
        }
        for (int i = 0; i < 2; i++) {
          actual.wheel[i] = (wheel_position[0][i] - wheel_position[ave_num - 1][i]) / (ave_num - 1) * 1000000 / SPEED_CONTROLLER_PERIOD_US + sum_accel * SPEED_CONTROLLER_PERIOD_US / 1000000 / 2;
        }
        actual.wheel2pole();
        actual.rot = mpu.gyro.z;
        actual.pole2wheel();
        for (int i = 0; i < 2; i++) {
          integral.wheel[i] += (actual.wheel[i] - target.wheel[i]) * SPEED_CONTROLLER_PERIOD_US / 1000000;
        }
        differential.trans = (accel[0] + accel[1] + accel[2]) / 3 - (target.trans - target_prev.trans) / SPEED_CONTROLLER_PERIOD_US * 1000000;
        differential.rot = 0;
        differential.pole2wheel();
        float pwm_value[2];
        for (int i = 0; i < 2; i++) {
          pwm_value[i] = Kp * (target.wheel[i] - actual.wheel[i]) + Kp * Ki * (0 - integral.wheel[i]) + Kp * Kd * (0 - differential.wheel[i]);
        }
        mt.drive(pwm_value[0], pwm_value[1]);

        position.theta += (actual_prev.rot + actual.rot) / 2 * SPEED_CONTROLLER_PERIOD_US / 1000000;
        position.x += (actual_prev.trans + actual.trans) / 2 * cos(position.theta) * SPEED_CONTROLLER_PERIOD_US / 1000000;
        position.y += (actual_prev.trans + actual.trans) / 2 * sin(position.theta) * SPEED_CONTROLLER_PERIOD_US / 1000000;

        actual_prev.trans = actual.trans;
        actual_prev.rot = actual.rot;
        target_prev.trans = target.trans;
      }
    }
};

extern SpeedController sc;

