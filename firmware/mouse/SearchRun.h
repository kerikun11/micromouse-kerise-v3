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

#define SEARCH_WALL_ATTACH_ENABLED  true
#define SEARCH_WALL_CUT_ENABLED     true
#define SEARCH_WALL_FRONT_ENABLED   false
#define SEARCH_WALL_AVOID_ENABLED   true

#define SEARCH_LOOK_AHEAD   12
#define SEARCH_PROP_GAIN    27

#define SEARCH_RUN_TASK_PRIORITY    3
#define SEARCH_RUN_STACK_SIZE       8192
#define SEARCH_RUN_PERIOD           1000

#define SEARCH_RUN_VELOCITY         240.0f
#define SEARCH_RUN_V_CURVE          240.0f
#define SEARCH_RUN_V_MAX            600.0f

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
      int look_ahead = SEARCH_LOOK_AHEAD;
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
    const float velocity = SEARCH_RUN_V_CURVE;
    const float straight = 5.0f;
  private:
    bool mirror;
    virtual int size() const {
      return 66;
    }
    virtual Position position(int index) const {
      static const float data[66 + 1][3] = {
        {0.0000000000, 0.0000000000, 0.0000000000}, {0.9999999926, 0.0000773152, 0.0003086046}, {1.9999991533, 0.0012208386, 0.0024200803}, {2.9999862627, 0.0060454113, 0.0079003866}, {3.9999042317, 0.0185252422, 0.0178727920}, {4.9995836606, 0.0434702426, 0.0328702678}, {5.9986670223, 0.0858893497, 0.0527654514}, {6.9965648523, 0.1503232120, 0.0767873824}, {7.9924843397, 0.2402355913, 0.1036228134}, {8.9855474634, 0.3575387753, 0.1316060125}, {9.9749291744, 0.5026540310, 0.1596625310}, {10.9598504769, 0.6754671406, 0.1877190495}, {11.9395361497, 0.8758420991, 0.2157755680}, {12.9132153208, 1.1036211226, 0.2438320865}, {13.8801215147, 1.3586249512, 0.2718886050}, {14.8394936084, 1.6406529254, 0.2999451235}, {15.7905762684, 1.9494830264, 0.3280016420}, {16.7326210354, 2.2848721543, 0.3560581605}, {17.6648866079, 2.6465562491, 0.3841146790}, {18.5866390333, 3.0342507041, 0.4121711975}, {19.4971527336, 3.4476504135, 0.4402277160}, {20.3957108952, 3.8864299293, 0.4682842345}, {21.2816064643, 4.3502438310, 0.4963407530}, {22.1541422290, 4.8387270063, 0.5243972715}, {23.0126312621, 5.3514950905, 0.5524537900}, {23.8563977390, 5.8881444796, 0.5805103085}, {24.6847774969, 6.4482527333, 0.6085668270}, {25.4971187237, 7.0313788618, 0.6366233455}, {26.2927819795, 7.6370639222, 0.6646798639}, {27.0711409235, 8.2648313203, 0.6927363824}, {27.8315827460, 8.9141868660, 0.7207929009}, {28.5735089838, 9.5846194128, 0.7488494194}, {29.2963358274, 10.2756011096, 0.7769059379}, {29.9994942217, 10.9865882082, 0.8049624564}, {30.6824306630, 11.7170212061, 0.8330189749}, {31.3446074863, 12.4663250549, 0.8610754934}, {31.9855036265, 13.2339099076, 0.8891320119}, {32.6046147173, 14.0191714937, 0.9171885304}, {33.2014533312, 14.8214919213, 0.9452450489}, {33.7755496421, 15.6402397153, 0.9733015674}, {34.3264517308, 16.4747703355, 1.0013580859}, {34.8537261177, 17.3244267715, 1.0294146044}, {35.3569577789, 18.1885402664, 1.0574711229}, {35.8357505544, 19.0664308867, 1.0855276414}, {36.2897275025, 19.9574075649, 1.1135841599}, {36.7185313255, 20.8607689543, 1.1416406784}, {37.1218246124, 21.7758038096, 1.1696971969}, {37.4992898733, 22.7017920395, 1.1977537154}, {37.8506299754, 23.6380049772, 1.2258102338}, {38.1755683152, 24.5837055712, 1.2538667523}, {38.4738491902, 25.5381493897, 1.2819232708}, {38.7452378736, 26.5005850279, 1.3099797893}, {38.9895206838, 27.4702551690, 1.3380363078}, {39.2065053397, 28.4463966696, 1.3660928263}, {39.3960210567, 29.4282409544, 1.3941493448}, {39.5579186838, 30.4150152769, 1.4222058633}, {39.6919981719, 31.4059533883, 1.4502623811}, {39.7983801811, 32.4002464178, 1.4779980248}, {39.8779145035, 33.3970504192, 1.5039081843}, {39.9331747718, 34.3955011822, 1.5264314840}, {39.9681150277, 35.3948770171, 1.5444516085}, {39.9875386677, 36.3946813128, 1.5574439951}, {39.9964559987, 37.3946388123, 1.5655447656}, {39.9994186253, 38.3946337576, 1.5695328390}, {39.9999182025, 39.3946335725, 1.5707275750}, {40.0000000000, 40.0000000000, 1.5707963268},
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
      static const char name[][32] = { "start_step", "start_init", "go_straight", "go_half", "turn_left_90", "turn_right_90", "turn_back", "return", "stop", };
      return name[action];
    }
    void enable() {
      printf("SearchRun Enabled\n");
      delete_task();
      create_task();
      //      lg.start();
    }
    void disable() {
      //      lg.end();
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
      printf("%s\tRel:(%06.1f, %06.1f, %06.3f)\n", name, sc.position.x, sc.position.y, sc.position.theta);
    }
  private:
    Position origin;
    std::queue<struct Operation> q;
    bool prev_wall[2];

    void wall_attach() {
#if SEARCH_WALL_ATTACH_ENABLED
      if (tof.getDistance() < 90) {
        portTickType xLastWakeTime = xTaskGetTickCount();
        int cnt = 0;
        while (1) {
          SpeedController::WheelParameter wp;
          const float gain = 0.5f;
          const float satu = 100.0f;
          wp.wheel[0] = -std::max(std::min(wd.wall_diff.front[0] * gain, satu), -satu);
          wp.wheel[1] = -std::max(std::min(wd.wall_diff.front[1] * gain, satu), -satu);
          wp.wheel2pole();
          if (fabs(wp.wheel[0]) + fabs(wp.wheel[1]) < 0.1f) break;
          sc.set_target(wp.trans, wp.rot);
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        }
        sc.set_target(0, 0);
        printPosition("wall_attach");
        sc.position.x = 0;  //< 直進方向の補正
        sc.position.theta = 0;  //< 直進方向の補正
        bz.play(Buzzer::SHORT);
      }
#endif
    }
    void wall_avoid(const float distance) {
#if SEARCH_WALL_AVOID_ENABLED
      const float gain = 0.0001f;
      const float satu = 0.25f;
      if (ref.side(0) > 60) sc.position.y += std::max(std::min(wd.wall_diff.side[0] * gain, satu), -satu);
      if (ref.side(1) > 60) sc.position.y -= std::max(std::min(wd.wall_diff.side[1] * gain, satu), -satu);
#endif
#if SEARCH_WALL_CUT_ENABLED
      for (int i = 0; i < 2; i++) {
        if (prev_wall[i] && !wd.wall[i] && sc.position.x > 30.0f) {
          const float prev_x = sc.position.x;
          if (distance > 90 - 1)
            sc.position.x = sc.position.x - ((int)sc.position.x) % 90 + 70 - 10;
          printf("WallCut[%d] X_ distance: %.0f, x: %.1f => %.1f\n", i, distance, prev_x, sc.position.x);
        }
        if (!prev_wall[i] && wd.wall[i] && sc.position.x > 30.0f) {
          const float prev_x = sc.position.x;
          if (distance > 90 - 1)
            sc.position.x = sc.position.x - ((int)sc.position.x) % 90 + 60 - 10;
          printf("WallCut[%d] _X distance: %.0f, x: %.1f => %.1f\n", i, distance, prev_x, sc.position.x);
        }
        prev_wall[i] = wd.wall[i];
      }
#endif
    }
    void wall_calib(const float velocity) {
#if SEARCH_WALL_FRONT_ENABLED
      if (wd.wall[2]) {
        float value = tof.getDistance() - (10.0f + tof.passedTimeMs()) / 1000.0f * velocity;
        float x = sc.position.x;
        sc.position.x = 90 - value;
        bz.play(Buzzer::SHORT);
        printf("FrontWallCalib: %.2f => %.2f\n", x, 90 - value);
      }
#endif
    }
    void turn(const float angle) {
      const float speed = 4 * M_PI;
      const float accel = 36 * M_PI;
      const float decel = 12 * M_PI;
      const float back_gain = 5.0f;
      int ms = 0;
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        if (fabs(sc.actual.rot) > speed) break;
        float delta = sc.position.x * cos(-sc.position.theta) - sc.position.y * sin(-sc.position.theta);
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
        float extra = angle - sc.position.theta;
        if (fabs(sc.actual.rot) < 0.1 && fabs(extra) < 0.1) break;
        float target_speed = sqrt(2 * decel * fabs(extra));
        float delta = sc.position.x * cos(-sc.position.theta) - sc.position.y * sin(-sc.position.theta);
        target_speed = (target_speed > speed) ? speed : target_speed;
        if (extra > 0) {
          sc.set_target(-delta * back_gain, target_speed);
        } else {
          sc.set_target(-delta * back_gain, -target_speed);
        }
      }
      sc.set_target(0, 0);
      sc.position.theta -= angle;               //< 移動した量だけ位置を更新
      sc.position = sc.position.rotate(-angle); //< 移動した量だけ位置を更新
      printPosition("Turn End");
    }
    void straight_x(const float distance, const float v_max, const float v_end) {
      const float accel = 2000;
      const float decel = 1500;
      int ms = 0;
      float v_start = sc.actual.trans;
      float T = 1.5f * (v_max - v_start) / accel;
      portTickType xLastWakeTime = xTaskGetTickCount();
      for (int i = 0; i < 2; i++) prev_wall[i] = wd.wall[i];
      while (1) {
        Position cur = sc.position;
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
        wall_avoid(distance);
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        ms++;
      }
      sc.set_target(v_end, 0);
      sc.position.x -= distance; //< 移動した量だけ位置を更新
      printPosition("Straight End");
    }
    template<class C>
    void trace(C tr, const float velocity) {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        if (tr.getRemain() < SEARCH_LOOK_AHEAD) break;
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        Position dir = tr.getNextDir(sc.position, velocity);
        sc.set_target(velocity, SEARCH_PROP_GAIN * dir.theta);
      }
      sc.set_target(velocity, 0);
      const Position end = tr.getEndPosition();
      sc.position = (sc.position - end).rotate(-end.theta);
      printPosition("Trace End");
    }
    void put_back() {
      const int max_v = 150;
      for (int i = 0; i < max_v; i++) {
        sc.set_target(-i, -sc.position.theta * 200.0f);
        delay(1);
      }
      for (int i = 0; i < 100; i++) {
        sc.set_target(-max_v, -sc.position.theta * 200.0f);
        delay(1);
      }
      sc.disable();
      mt.drive(-100, -100);
      delay(200);
      sc.enable(true);
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
    virtual void task() {
      const float velocity = SEARCH_RUN_VELOCITY;
      const float v_max = SEARCH_RUN_V_MAX;
      const float ahead_length = 9.0f;
      sc.enable();
      while (1) {
        //** SearchActionがキューされるまで直進で待つ
        {
          S90 tr;
          portTickType xLastWakeTime = xTaskGetTickCount();
          while (q.empty()) {
            vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
            Position cur = sc.position;
            float theta = atan2f(-cur.y, SEARCH_LOOK_AHEAD) - cur.theta;
            sc.set_target(velocity, SEARCH_PROP_GAIN * theta);
            wall_avoid(0);
          }
        }
        struct Operation operation = q.front();
        enum ACTION action = operation.action;
        int num = operation.num;
        printf("Action: %d %s\n", num, action_string(action));
        switch (action) {
          case START_STEP:
            sc.position.reset();
            straight_x(SEGMENT_WIDTH - MACHINE_TAIL_LENGTH - WALL_THICKNESS / 2 + ahead_length, velocity, velocity);
            break;
          case START_INIT:
            straight_x(SEGMENT_WIDTH / 2 - ahead_length, velocity, 0);
            wall_attach();
            turn(M_PI / 2);
            wall_attach();
            turn(M_PI / 2);
            put_back();
            mt.free();
            while (!q.empty()) q.pop();
            while (1) delay(1000);
          case GO_STRAIGHT:
            straight_x(SEGMENT_WIDTH * num, v_max, velocity);
            break;
          case GO_HALF:
            straight_x(SEGMENT_WIDTH / 2 * num, v_max, velocity);
            break;
          case TURN_LEFT_90:
            for (int i = 0; i < num; i++) {
              S90 tr(false);
              wall_calib(velocity);
              straight_x(tr.straight - ahead_length, velocity, tr.velocity);
              trace(tr, tr.velocity);
              straight_x(tr.straight + ahead_length, tr.velocity, velocity);
            }
            break;
          case TURN_RIGHT_90:
            for (int i = 0; i < num; i++) {
              S90 tr(true);
              wall_calib(velocity);
              straight_x(tr.straight - ahead_length, velocity, tr.velocity);
              trace(tr, tr.velocity);
              straight_x(tr.straight + ahead_length, tr.velocity, velocity);
            }
            break;
          case TURN_BACK:
            straight_x(SEGMENT_WIDTH / 2 - ahead_length, velocity, 0);
            uturn();
            straight_x(SEGMENT_WIDTH / 2 + ahead_length, velocity, velocity);
            break;
          case RETURN:
            uturn();
            break;
          case STOP:
            straight_x(SEGMENT_WIDTH / 2 - ahead_length, velocity, 0);
            wall_attach();
            sc.disable();
            while (!q.empty()) q.pop();
            while (1) delay(1000);
            break;
        }
        q.pop();
        printPosition("End");
      }
      while (1) delay(1000);
    }
};

extern SearchRun sr;

