#pragma once

#include <Arduino.h>
#include <cstdlib>

#include "motor.h"
extern Motor mt;
#include "imu.h"
extern IMU imu;
#include "encoder.h"
extern Encoder enc;

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

template<typename T, size_t _size>
class  Accumulator {
  public:
    Accumulator(const T& value = T()) {
      buffer = (T*)std::malloc(_size * sizeof(T));
      head = 0;
      clear(value);
    }
    void clear(const T& value = T()) {
      for (int i = 0; i < _size; i++) buffer[i] = value;
    }
    void push(const T& value) {
      head = (head + 1) % _size;
      buffer[head] = value;
    }
    const T& operator[](const size_t index) const {
      return buffer[(_size + head - index) % _size];
    }
    const T& average(const int num) const {
      T& sum = T();
      for (int i = 0; i < num; i++) {
        sum += this[i];
      }
      return sum / num;
    }
    size_t size() const {
      return _size;
    }
  private:
    T* buffer;
    size_t head;
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
    WheelParameter acconly;
    WheelParameter proportional;
    WheelParameter integral;
    WheelParameter differential;
    float Kp = SPEED_CONTROLLER_KP;
    float Ki = SPEED_CONTROLLER_KI;
    float Kd = SPEED_CONTROLLER_KD;
    Position position;
    int ave_num;

  public:
    SpeedController() {
      enabled = false;
      reset();
      xTaskCreate([](void* obj) {
        static_cast<SpeedController*>(obj)->task();
      }, "SpeedController", SPEED_CONTROLLER_STACK_SIZE, this, SPEED_CONTROLLER_TASK_PRIORITY, NULL);
    }
    void enable(const bool& reset_position = true) {
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
    void set_target(const float& trans, const float& rot) {
      target.trans = trans;
      target.rot = rot;
      target.pole2wheel();
    }
  private:
    bool enabled = false;
    static const int acc_num = 16;
    Accumulator<float, acc_num> wheel_position[2];
    Accumulator<float, acc_num> accel;
    Accumulator<float, acc_num> gyro;
    WheelParameter actual_prev;
    WheelParameter target_prev;

    void reset() {
      target.clear();
      actual.clear();
      integral.clear();
      differential.clear();
      actual_prev.clear();
      target_prev.clear();
      for (int i = 0; i < 2; i++) wheel_position[i].clear(enc.position(i));
      accel.clear(imu.accel.y);
      gyro.clear(imu.gyro.z);
    }
    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        if (enabled == false) continue;

        // 最新のデータの追加
        for (int i = 0; i < 2; i++) wheel_position[i].push(enc.position(i));
        accel.push(imu.accel.y);
        gyro.push(imu.gyro.z);

        // LPFのサンプル数を指定
        //        ave_num = std::min(acc_num, static_cast<int>(PI * MACHINE_GEAR_RATIO * MACHINE_WHEEL_DIAMETER / ((1.0f + fabs(target.trans)) * SPEED_CONTROLLER_PERIOD_US / 1000000)));
        ave_num = acc_num;

        float sum_accel = 0.0f;
        for (int j = 0; j < ave_num - 1; j++) sum_accel += accel[j];

        for (int i = 0; i < 2; i++) {
          actual.wheel[i] = (wheel_position[i][0] - wheel_position[i][ave_num - 1]) / (ave_num - 1) * 1000000 / SPEED_CONTROLLER_PERIOD_US;
          enconly.wheel[i] = (wheel_position[i][0] - wheel_position[i][1]) * 1000000 / SPEED_CONTROLLER_PERIOD_US;
        }
        acconly.trans = sum_accel * SPEED_CONTROLLER_PERIOD_US / 1000000 / 2;

        enconly.wheel2pole();
        actual.wheel2pole();
        actual.trans += sum_accel * SPEED_CONTROLLER_PERIOD_US / 1000000 / 2;
        actual.rot = imu.gyro.z;
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

