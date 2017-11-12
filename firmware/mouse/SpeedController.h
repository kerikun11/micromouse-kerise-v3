#pragma once

#include <Arduino.h>
#include <deque>

#include "motor.h"
#include "axis.h"
#include "encoder.h"

class Position {
  public:
    float x, y, theta;
  public:
    Position(const float x = 0, const float y = 0, const float theta = 0) : x(x), y(y), theta(theta) {}
    Position(const float pos[3]) : x(pos[0]), y(pos[1]), theta(pos[2]) {}

    void reset() {
      x = 0; y = 0; theta = 0;
    }
    void set(const float x = 0, const float y = 0, const float theta = 0) {
      this->x = x; this->y = y; this->theta = theta;
    }
    const Position rotate(const float angle) const {
      //      float _x = x;
      //      float _y = y;
      //      x = _x * cos(angle) - _y * sin(angle);
      //      y = _x * sin(angle) + _y * cos(angle);
      //      return *this;
      return Position(x * cos(angle) - y * sin(angle), x * sin(angle) + y * cos(angle), theta);
    }
    float getNorm() const {
      return sqrt(x * x + y * y);
    }
    Position mirror_x() const {
      return Position(x, -y, -theta);
    }

    Position operator=(const Position &obj) {
      x = obj.x; y = obj.y; theta = obj.theta; return *this;
    }
    Position operator+() const {
      return Position(x, y, theta);
    }
    Position operator+(const Position &obj) const {
      return Position(x + obj.x, y + obj.y, theta + obj.theta);
    }
    Position operator+=(const Position &obj) {
      x += obj.x; y += obj.y; theta += obj.theta; return *this;
    }
    Position operator-() const {
      return Position(-x, -y, -theta);
    }
    Position operator-(const Position &obj) const {
      return Position(x - obj.x, y - obj.y, theta - obj.theta);
    }
    Position operator-=(const Position &obj) {
      x -= obj.x; y -= obj.y; theta -= obj.theta; return *this;
    }
    inline void print(const char* name = "Pos") {
      printf("%s: (%.1f,\t%.1f,\t%.3f)\n", name, x, y, theta);
    }
};

#define SPEED_CONTROLLER_TASK_PRIORITY  4
#define SPEED_CONTROLLER_STACK_SIZE     4096

#define SPEED_CONTROLLER_KP   1.2f
#define SPEED_CONTROLLER_KI   144.0f
#define SPEED_CONTROLLER_KD   0.01f

#define SPEED_CONTROLLER_PERIOD_US  1000

class SpeedController {
  public:
    struct WheelParameter {
      public:
        float trans;    //< translation [mm]
        float rot;      //< rotation [rad]
        float wheel[2]; //< wheel position [mm], wheel[0]:left, wheel[1]:right
      public:
        WheelParameter() {}
        WheelParameter(const float trans, const float rot) : trans(trans), rot(rot) {}
        WheelParameter(const WheelParameter& obj) : trans(obj.trans), rot(obj.rot), wheel( {
          obj.wheel[0], obj.wheel[1]
        }) {}
        void pole2wheel() {
          wheel[0] = trans - MACHINE_ROTATION_RADIUS * rot;
          wheel[1] = trans + MACHINE_ROTATION_RADIUS * rot;
        }
        void wheel2pole() {
          rot = (wheel[1] - wheel[0]) / 2.0f / MACHINE_ROTATION_RADIUS;
          trans = (wheel[1] + wheel[0]) / 2.0f;
        }
        void clear() {
          trans = 0;
          rot = 0;
          wheel[0] = 0;
          wheel[1] = 0;
        }
    };
    WheelParameter target;
    WheelParameter actual;
    WheelParameter enconly;
    WheelParameter proportional;
    WheelParameter integral;
    WheelParameter differential;
    float Kp = SPEED_CONTROLLER_KP;
    float Ki = SPEED_CONTROLLER_KI;
    float Kd = SPEED_CONTROLLER_KD;
    Position position;

  public:
    SpeedController() {
      enabled = false;
      reset();
      xTaskCreate([](void* obj) {
        static_cast<SpeedController*>(obj)->task();
      }, "SpeedController", SPEED_CONTROLLER_STACK_SIZE, this, SPEED_CONTROLLER_TASK_PRIORITY, NULL);
    }
    void enable(bool reset_position = true) {
      reset();
      if (reset_position) position.reset();
      enabled = true;
      printf("Speed Controller Enabled\n");
    }
    void disable() {
      enabled = false;
      delay(2);
      mt.free();
      printf("Speed Controller disabled\n");
    }
    void set_target(float trans, float rot) {
      target.trans = trans;
      target.rot = rot;
      target.pole2wheel();
    }
  private:
    bool enabled = false;
    static const int ave_num = 5;
    float wheel_position[ave_num][2];
    float accel[ave_num];
    float gyro[ave_num];
    WheelParameter actual_prev;
    WheelParameter target_prev;

    void reset() {
      target.clear();
      actual.clear();
      integral.clear();
      differential.clear();
      actual_prev.clear();
      target_prev.clear();
      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < ave_num; j++) {
          wheel_position[j][i] = enc.position(i);
          accel[j] = 0;
          gyro[j] = 0;
        }
      }
    }
    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        if (enabled == false) continue;

        for (int i = 0; i < 2; i++) {
          for (int j = ave_num - 1; j > 0; j--) {
            wheel_position[j][i] = wheel_position[j - 1][i];
          }
          wheel_position[0][i] = enc.position(i);
        }
        for (int j = ave_num - 1; j > 0; j--) {
          accel[j] = accel[j - 1];
          gyro[j] = gyro[j - 1];
        }
        accel[0] = axis.accel.y;
        gyro[0] = axis.gyro.z;
        float sum_accel = 0.0f;
        for (int j = 0; j < ave_num - 1; j++) sum_accel += accel[j];
        for (int i = 0; i < 2; i++) {
          actual.wheel[i] = (wheel_position[0][i] - wheel_position[ave_num - 1][i]) / (ave_num - 1) * 1000000 / SPEED_CONTROLLER_PERIOD_US + sum_accel * SPEED_CONTROLLER_PERIOD_US / 1000000 / 2;
          //          actual.wheel[i] = (wheel_position[0][i] - wheel_position[ave_num - 1][i]) / (ave_num - 1)  * 1000000 / SPEED_CONTROLLER_PERIOD_US;
          enconly.wheel[i] = (wheel_position[0][i] - wheel_position[1][i]) * 1000000 / SPEED_CONTROLLER_PERIOD_US;
        }
        enconly.wheel2pole();
        //        enconly.rot = axis.gyro.z;
        //        enconly.pole2wheel();
        actual.wheel2pole();
        actual.rot = axis.gyro.z;
        actual.pole2wheel();
        for (int i = 0; i < 2; i++) {
          integral.wheel[i] += (target.wheel[i] - actual.wheel[i]) * SPEED_CONTROLLER_PERIOD_US / 1000000;
          proportional.wheel[i] = target.wheel[i] - actual.wheel[i];
        }
        //        differential.trans = (target.trans - target_prev.trans) / SPEED_CONTROLLER_PERIOD_US * 1000000 - accel[0];
        differential.trans = (target.trans - target_prev.trans) / SPEED_CONTROLLER_PERIOD_US * 1000000;
        differential.rot = (target.rot - target_prev.rot) / SPEED_CONTROLLER_PERIOD_US * 1000000 - (gyro[0] - gyro[1]) / SPEED_CONTROLLER_PERIOD_US * 1000000;
        differential.pole2wheel();
        float pwm_value[2];
        for (int i = 0; i < 2; i++) pwm_value[i] = Kp * proportional.wheel[i] + Ki * integral.wheel[i] + Kd * differential.wheel[i];
        mt.drive(pwm_value[0], pwm_value[1]);

        proportional.wheel2pole();
        integral.wheel2pole();

        position.theta += (actual_prev.rot + actual.rot) / 2 * SPEED_CONTROLLER_PERIOD_US / 1000000;
        position.x += (actual_prev.trans + actual.trans) / 2 * cos(position.theta) * SPEED_CONTROLLER_PERIOD_US / 1000000;
        position.y += (actual_prev.trans + actual.trans) / 2 * sin(position.theta) * SPEED_CONTROLLER_PERIOD_US / 1000000;
        actual_prev = actual;
        target_prev = target;
      }
    }
};

extern SpeedController sc;

