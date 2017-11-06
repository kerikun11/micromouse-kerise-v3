#pragma once

#include <Arduino.h>
#include <vector>
#include <queue>
#include "TaskBase.h"
#include "config.h"
#include "logger.h"
#include "encoder.h"
#include "motor.h"
#include "axis.h"
#include "reflector.h"
#include "WallDetector.h"
#include "SpeedController.h"

#define SEARCH_WALL_ATTACH_ENABLED     true
#define SEARCH_WALL_AVOID_ENABLED      false
#define SEARCH_WALL_AVOID_GAIN         0.0000005f

#define SEARCH_LOOK_AHEAD   5
#define SEARCH_PROP_GAIN    30

#define SEARCH_RUN_TASK_PRIORITY   3
#define SEARCH_RUN_STACK_SIZE      8192
#define SEARCH_RUN_PERIOD          1000

//#define printf  lg.printf

class SearchTrajectory {
  public:
    SearchTrajectory() {
      reset();
    }
    virtual ~SearchTrajectory() {}
    void reset() {
      last_index = -SEARCH_LOOK_AHEAD;
    }
    Position getNextDir(const Position &cur, float velocity) {
      int index_cur = getNextIndex(cur);
      int look_ahead = SEARCH_LOOK_AHEAD + velocity / 100;
      Position dir = (getPosition(index_cur + look_ahead) - cur).rotate(-cur.theta);
      dir.theta = atan2f(dir.y, dir.x);
      return dir;
    }
    float getRemain() const {
      return (getSize() - last_index) * interval;
    }
    Position getEndPosition() const {
      return getPosition(getSize());
    }
  protected:
    int last_index;
    const float interval = 1.0f;
    virtual int size() const {
      return 1;
    }
    virtual Position position(const int index) const {
      return Position(index * interval, 0, 0);
    }
    int getSize() const {
      return size();
    }
    Position getPosition(const int index) const {
      return position(index);
    }
    int getNextIndex(const Position& pos) {
      for (int i = last_index;; i++) {
        Position target = getPosition(i);
        Position dir = (target - pos).rotate(-target.theta);
        if (dir.x > 0) {
          last_index = i;
          return last_index;
        }
      }
      return last_index;
    }
};

class S90: public SearchTrajectory {
  public:
    S90(bool mirror = false) : mirror(mirror) {}
    const float velocity = 120.0f;
    const float straight = 5.0f;
  private:
    bool mirror;
    virtual int size() const {
      return 69;
    }
    virtual Position position(int index) const {
      static const float data[69 + 1][3] = {
        {0.0000000000, 0.0000000000, 0.0000000000}, {0.9999999962, 0.0000094642, 0.0000378499}, {1.9999999773, 0.0001511747, 0.0003022989}, {2.9999997995, 0.0007644404, 0.0010174506}, {3.9999983085, 0.0024095148, 0.0024024453}, {4.9999921290, 0.0058630426, 0.0046690468}, {5.9999722739, 0.0121091045, 0.0080193101}, {6.9999189950, 0.0223262955, 0.0126433563}, {7.9997965880, 0.0378780606, 0.0187172793}, {8.9995430023, 0.0602954548, 0.0264012060}, {9.9990595129, 0.0912578826, 0.0358375335}, {10.9982003001, 0.1325784102, 0.0471493602}, {11.9967555602, 0.1861785380, 0.0604391311}, {12.9944384799, 0.2540640374, 0.0757875088}, {13.9908713857, 0.3383049548, 0.0932524857}, {14.9855679304, 0.4410028843, 0.1128687454}, {15.9779221178, 0.5642634348, 0.1346472802}, {16.9671963943, 0.7101708672, 0.1585752707}, {17.9525117713, 0.8807498286, 0.1846162266}, {18.9328440736, 1.0779370372, 0.2127103886}, {19.9070217554, 1.3035522259, 0.2427753866}, {20.8737321334, 1.5592597075, 0.2747071467}, {21.8315285645, 1.8465435433, 0.3083810374}, {22.7788447518, 2.1666798452, 0.3436532420}, {23.7140200547, 2.5207072335, 0.3803623424}, {24.6353182501, 2.9094107281, 0.4183310968}, {25.5409593801, 3.3333037881, 0.4573683914}, {26.4291594281, 3.7926172309, 0.4972713452}, {27.2981564990, 4.2872958982, 0.5378275442}, {28.1462563341, 4.8170001824, 0.5788173804}, {28.9718709563, 5.3811090832, 0.6200164878}, {29.7735329825, 5.9787681433, 0.6612453799}, {30.5498799234, 6.6089615927, 0.7024742720}, {31.2995923145, 7.2706183668, 0.7437031641}, {32.0213959591, 7.9626139265, 0.7849320562}, {32.7140640938, 8.6837721694, 0.8261609483}, {33.3764194730, 9.4328674290, 0.8673898404}, {34.0073363702, 10.2086265572, 0.9086187325}, {34.6057424913, 11.0097310888, 0.9498476246}, {35.1706259794, 11.8348160331, 0.9910488666}, {35.7011163327, 12.6824252427, 1.0320459270}, {36.1965914714, 13.5509669668, 1.0726143799}, {36.6567063307, 14.4387521442, 1.1125344827}, {37.0813985363, 15.3440203128, 1.1515936324}, {37.4708935550, 16.2649825872, 1.1895887073}, {37.8256971959, 17.1998636497, 1.2263283039}, {38.1465883403, 18.1469251523, 1.2616348444}, {38.4346030618, 19.1045012487, 1.2953465326}, {38.6910084063, 20.0710266805, 1.3273191357}, {38.9172851174, 21.0450509275, 1.3574275722}, {39.1150972597, 22.0252572879, 1.3855672900}, {39.2862561207, 23.0104719004, 1.4116554172}, {39.4326983093, 23.9996666706, 1.4356316752}, {39.5564497952, 24.9919603265, 1.4574590420}, {39.6595885621, 25.9866109199, 1.4771241604}, {39.7442228222, 26.9830098484, 1.4946374848}, {39.8124565654, 27.9806698637, 1.5100331663}, {39.8663563879, 28.9792085621, 1.5233686764}, {39.9079325713, 29.9783382261, 1.5347241740}, {39.9391098451, 30.9778488363, 1.5442016230}, {39.9617006069, 31.9775909539, 1.5519236692}, {39.9773899210, 32.9774661280, 1.5580322898}, {39.9877127586, 33.9774122382, 1.5626872297}, {39.9940343177, 34.9773916692, 1.5660642423}, {39.9975393246, 35.9773852398, 1.5683531544}, {39.9992162983, 36.9773838523, 1.5697557753}, {39.9998455949, 37.9773835975, 1.5704836745}, {39.9999934554, 38.9773835831, 1.5707558520}, {40.0000038070, 39.9773835832, 1.5707963264}, {40.0000000000, 40.0000000078, 1.5707963268},
      };
      Position ret;
      if (index < 0) {
        ret = Position(0 + interval * index, 0, 0);
      } else if (index > size() - 1) {
        Position end(data[size()][0], data[size()][1], data[size()][2]);
        ret = end + Position((index - size()) * interval * cos(end.theta), (index - size()) * interval * sin(end.theta), 0);
      } else {
        ret = Position(data[index][0], data[index][1], data[index][2]);
      }
      if (mirror) return ret.mirror_x();
      return ret;
    }
};

class SearchRun: TaskBase {
  public:
    SearchRun() : TaskBase("SearchRun", SEARCH_RUN_TASK_PRIORITY, SEARCH_RUN_STACK_SIZE) {}
    virtual ~SearchRun() {}
    enum ACTION {
      START_STEP, START_INIT, GO_STRAIGHT, GO_HALF, TURN_LEFT_90, TURN_RIGHT_90, TURN_BACK, RETURN, STOP,
    };
    struct Operation {
      enum ACTION action;
      int num;
    };
    const char* action_string(enum ACTION action) {
      static const char name[][32] =
      { "start_step", "start_init", "go_straight", "go_half", "turn_left_90", "turn_right_90", "turn_back", "return", "stop", };
      return name[action];
    }
    void enable() {
      printf("SearchRun Enabled\n");
      delete_task();
      create_task();
    }
    void disable() {
      delete_task();
      sc.disable();
      while (q.size()) {
        q.pop();
      }
      printf("SearchRun Disabled\n");
    }
    void set_action(enum ACTION action, int num = 1) {
      struct Operation operation;
      operation.action = action;
      operation.num = num;
      q.push(operation);
    }
    int actions() const {
      return q.size();
    }
    void waitForEnd() const {
      while (actions()) {
        delay(1);
      }
    }
    void printPosition(const char* name) const {
      printf("%s\tRel:(%06.1f, %06.1f, %06.3f)\n", name, getRelativePosition().x, getRelativePosition().y, getRelativePosition().theta);
    }
    Position getRelativePosition() const {
      return (sc.getPosition() - origin).rotate(-origin.theta);
    }
    void updateOrigin(Position passed) {
      origin += passed.rotate(origin.theta);
    }
    void setPosition(Position pos = Position(0, 0, 0)) {
      origin = pos;
      sc.getPosition() = pos;
    }
    void fixPosition(Position pos) {
      sc.getPosition() -= pos;
    }
  private:
    Position origin;
    std::queue<struct Operation> q;

    void wall_avoid(const float distance) {
#if SEARCH_WALL_AVOID_ENABLED
      const float gain = SEARCH_WALL_AVOID_GAIN;
      if (wd.getWall(0)) {
        float x = -wd.getDiff().side[0] * gain * sc.actual.trans;
        fixPosition(Position(0, x, 0).rotate(origin.theta));
      }
      if (wd.getWall(1)) {
        fixPosition(Position(0, wd.getDiff().side[1] * gain * sc.actual.trans, 0).rotate(origin.theta));
      }
#endif
#if 0
      if (tof.passedTimeMs() == 0) {
        if (tof.getDistance() > 90 && tof.getDistance() < 150) {
          float value = tof.getDistance() - 10.0f / 1000.0f * sc.actual.trans;
          if (abs(distance - SEGMENT_WIDTH / 2) < 1.0f) {
            fixPosition(Position(getRelativePosition().x - (135 - value), 0, 0).rotate(origin.theta));
            bz.play(Buzzer::SHORT);
          } else if (abs(distance - SEGMENT_WIDTH) < 1.0f) {
            fixPosition(Position(getRelativePosition().x - (180 - value), 0, 0).rotate(origin.theta));
            bz.play(Buzzer::SHORT);
          }
        }
      }
#endif
    }
    void wall_attach() {
#if SEARCH_WALL_ATTACH_ENABLED
      if (wd.getWall(2)) {
        portTickType xLastWakeTime = xTaskGetTickCount();
        int cnt = 0;
        while (1) {
          float trans = -(wd.getDiff().front[0] + wd.getDiff().front[1]) * 0.4f; //< 0.5
          float rot = (wd.getDiff().front[0] - wd.getDiff().front[1]) * 0.05f;
          const float trans_sat = 100;
          const float rot_sat = 1 * PI;
          if (trans > trans_sat) trans = trans_sat;
          else if (trans < -trans_sat)trans = -trans_sat;
          if (rot > rot_sat) rot = rot_sat;
          else if (rot < -rot_sat)rot = -rot_sat;
          if (fabs(trans) < 0.01f && fabs(rot) < 0.01f) break;
          sc.set_target(trans, rot);
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
          if (cnt++ % 100 == 0) printf("trans: %f, rot:%f\n", rot, trans);
        }
        sc.set_target(0, 0);
        printPosition("wall_attach");
        fixPosition(Position(getRelativePosition().x, 0, getRelativePosition().theta).rotate(origin.theta));
        bz.play(Buzzer::SELECT);
      }
#endif
    }
    void turn(const float angle) {
      const float speed = 3 * M_PI;
      const float accel = 36 * M_PI;
      const float decel = 12 * M_PI;
      const float back_gain = 5.0f;
      int ms = 0;
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        if (fabs(sc.actual.rot) > speed) break;
        float delta = getRelativePosition().x * cos(-getRelativePosition().theta) - getRelativePosition().y * sin(-getRelativePosition().theta);
        if (angle > 0) {
          sc.set_target(-delta * back_gain, ms / 1000.0f * accel);
        } else {
          sc.set_target(-delta * back_gain, -ms / 1000.0f * accel);
        }
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        ms++;
      }
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        float extra = angle - getRelativePosition().theta;
        if (fabs(sc.actual.rot) < 0.1 && abs(extra) < 0.1) break;
        float target_speed = sqrt(2 * decel * fabs(extra));
        float delta = getRelativePosition().x * cos(-getRelativePosition().theta) - getRelativePosition().y * sin(-getRelativePosition().theta);
        target_speed = (target_speed > speed) ? speed : target_speed;
        if (extra > 0) {
          sc.set_target(-delta * back_gain, target_speed);
        } else {
          sc.set_target(-delta * back_gain, -target_speed);
        }
      }
      sc.set_target(0, 0);
      updateOrigin(Position(0, 0, angle));
      printPosition("Turn End");
    }
    void straight_x(const float distance, const float v_max, const float v_end, bool avoid) {
      const float accel = 3000;
      const float decel = 1500;
      int ms = 0;
      float v_start = sc.actual.trans;
      float T = 1.5f * (v_max - v_start) / accel;
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        Position cur = getRelativePosition();
        if (v_end >= 1.0f && cur.x > distance - SEARCH_LOOK_AHEAD) break;
        if (v_end < 1.0f && cur.x > distance - 1.0f) break;
        float extra = distance - cur.x;
        float velocity_a = v_start + (v_max - v_start) * 6.0f * (-1.0f / 3 * pow(ms / 1000.0f / T, 3) + 1.0f / 2 * pow(ms / 1000.0f / T, 2));
        float velocity_d = sqrt(2 * decel * fabs(extra) + v_end * v_end);
        float velocity = v_max;
        if (velocity > velocity_d) velocity = velocity_d;
        if (ms / 1000.0f < T && velocity > velocity_a) velocity = velocity_a;
        float theta = atan2f(-cur.y, SEARCH_LOOK_AHEAD) - cur.theta;
        sc.set_target(velocity, SEARCH_PROP_GAIN * theta);
        if (avoid) wall_avoid(distance);
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        ms++;
      }
      sc.set_target(v_end, 0);
      updateOrigin(Position(distance, 0, 0));
      printPosition("Straight End");
    }
    template<class C>
    void trace(C tr, const float velocity) {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        if (tr.getRemain() < SEARCH_LOOK_AHEAD) break;
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        Position dir = tr.getNextDir(getRelativePosition(), velocity);
        sc.set_target(velocity, SEARCH_PROP_GAIN * dir.theta);
      }
      sc.set_target(velocity, 0);
      updateOrigin(tr.getEndPosition());
      printPosition("Trace End");
    }
    void put_back() {
      const int max_v = 150;
      for (int i = 0; i < max_v; i++) {
        sc.set_target(-i, -getRelativePosition().theta * 200.0f);
        delay(1);
      }
      for (int i = 0; i < 100; i++) {
        sc.set_target(-max_v, -getRelativePosition().theta * 200.0f);
        delay(1);
      }
      sc.disable();
      mt.drive(-100, -100);
      delay(200);
      sc.enable();
      updateOrigin(Position(-SEGMENT_WIDTH / 2 + MACHINE_TAIL_LENGTH + WALL_THICKNESS / 2, 0, 0));
      setPosition(origin);
      fixPosition(Position(getRelativePosition().x, 0, getRelativePosition().theta).rotate(origin.theta));
    }
    void uturn() {
      if (axis.angle.z > 0) {
        wall_attach();
        turn(-M_PI / 2);
        wall_attach();
        turn(-M_PI / 2);
      } else {
        wall_attach();
        turn(M_PI / 2);
        wall_attach();
        turn(M_PI / 2);
      }
    }
    void wall_calib(const float velocity) {
      if (wd.getWall(2)) {
        float value = tof.getDistance() - (10.0f + tof.passedTimeMs()) / 1000.0f * velocity;
        fixPosition(Position(getRelativePosition().x - (90 - value), 0, 0).rotate(origin.theta));
        bz.play(Buzzer::SHORT);
      }
    }
    virtual void task() {
      const float velocity = 200;
      const float v_max = 360;
      const float ahead_length = 0.0f;
      sc.enable();
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        while (q.empty()) {
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
          S90 tr;
          Position cur = getRelativePosition();
          const float decel = 1200;
          //          float extra = tr.straight - ahead_length - cur.x - SEARCH_LOOK_AHEAD;
          float extra = tr.straight - ahead_length - cur.x;
          float v = sqrt(2 * decel * fabs(extra));
          if (v > velocity) v = velocity;
          if (extra < 0) v = -v;
          float theta = atan2f(-cur.y, SEARCH_LOOK_AHEAD) - cur.theta;
          sc.set_target(v, SEARCH_PROP_GAIN * theta);
          //          wall_avoid(0);
        }
        struct Operation operation = q.front();
        enum ACTION action = operation.action;
        int num = operation.num;
        printf("%s: %d\n", action_string(action), num);
        printPosition("Start");
        switch (action) {
          case START_STEP:
            setPosition();
            // なぜか振動するので，START_STEPにはwall_avoidを入れない
            //            straight_x(SEGMENT_WIDTH - MACHINE_TAIL_LENGTH - WALL_THICKNESS / 2 + ahead_length, velocity, velocity, true);
            straight_x(SEGMENT_WIDTH - MACHINE_TAIL_LENGTH - WALL_THICKNESS / 2 + ahead_length, velocity, velocity, false);
            break;
          case START_INIT:
            straight_x(SEGMENT_WIDTH / 2 - ahead_length, velocity, 0, true);
            wall_attach();
            turn(M_PI / 2);
            wall_attach();
            turn(M_PI / 2);
            put_back();
            mt.free();
            while (q.size()) {
              q.pop();
            }
            while (1) {
              delay(1000);
            }
          case GO_STRAIGHT:
            straight_x(SEGMENT_WIDTH * num, v_max, velocity, true);
            break;
          case GO_HALF:
            straight_x(SEGMENT_WIDTH / 2 * num, v_max, velocity, true);
            break;
          case TURN_LEFT_90:
            for (int i = 0; i < num; i++) {
              S90 tr(false);
              wall_calib(velocity);
              straight_x(tr.straight - ahead_length, velocity, tr.velocity, true);
              trace(tr, tr.velocity);
              straight_x(tr.straight + ahead_length, tr.velocity, velocity, true);
            }
            break;
          case TURN_RIGHT_90:
            for (int i = 0; i < num; i++) {
              S90 tr(true);
              wall_calib(velocity);
              straight_x(tr.straight - ahead_length, velocity, tr.velocity, true);
              trace(tr, tr.velocity);
              straight_x(tr.straight + ahead_length, tr.velocity, velocity, true);
            }
            break;
          case TURN_BACK:
            straight_x(SEGMENT_WIDTH / 2 - ahead_length, velocity, 0, true);
            uturn();
            straight_x(SEGMENT_WIDTH / 2 + ahead_length, velocity, velocity, true);
            break;
          case RETURN:
            uturn();
            break;
          case STOP:
            straight_x(SEGMENT_WIDTH / 2 - ahead_length, velocity, 0, true);
            wall_attach();
            break;
        }
        q.pop();
        printPosition("End");
      }
      while (1) {
        delay(1000);
      }
    }
};

extern SearchRun sr;

