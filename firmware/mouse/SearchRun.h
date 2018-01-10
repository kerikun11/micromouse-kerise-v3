#pragma once

#include <Arduino.h>
#include <vector>
#include <queue>
#include "TaskBase.h"
#include "config.h"

/* Hardware */
#include "buzzer.h"
extern Buzzer bz;
#include "button.h"
extern Button btn;
#include "led.h"
extern LED led;
#include "motor.h"
extern Motor mt;
#include "fan.h"
extern Fan fan;
#include "imu.h"
extern IMU imu;
#include "encoder.h"
extern Encoder enc;
#include "reflector.h"
extern Reflector ref;
#include "tof.h"
extern ToF tof;

/* Software */
#include "SpeedController.h"
extern SpeedController sc;
#include "WallDetector.h"
extern WallDetector wd;
#include "Logger.h"
extern Logger lg;

#define SEARCH_WALL_ATTACH_ENABLED  true
#define SEARCH_WALL_CUT_ENABLED     true
#define SEARCH_WALL_FRONT_ENABLED   true
#define SEARCH_WALL_AVOID_ENABLED   true

#define SEARCH_END_REMAIN           3
#define SEARCH_ST_LOOK_AHEAD(v)     (6+2*v/100)
#define SEARCH_ST_FB_GAIN           20
#define SEARCH_CURVE_FB_GAIN        2.0f

#define ahead_length                0

#define SEARCH_RUN_TASK_PRIORITY    3
#define SEARCH_RUN_STACK_SIZE       8192
#define SEARCH_RUN_PERIOD           1000

#define SEARCH_RUN_VELOCITY         240.0f
#define SEARCH_RUN_V_CURVE          240.0f
#define SEARCH_RUN_V_MAX            1200.0f

//#define printf  lg.printf

class SearchTrajectory {
  public:
    SearchTrajectory() {
      reset();
    }
    virtual ~SearchTrajectory() {}
    void reset() {
      last_index = -SEARCH_END_REMAIN;
    }
    Position getNextDir(const Position &cur, const float velocity) {
      int index_cur = getNextIndex(cur);
      Position dir = (getPosition(index_cur + 3) - cur).rotate(-cur.theta);
      float dt = 1.0f / velocity;
      float ff = (getPosition(last_index + 1).theta - getPosition(last_index).theta) / dt;
      dir.theta = ff + SEARCH_CURVE_FB_GAIN * atan2f(dir.y, dir.x);
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
      return 64;
    }
    virtual Position position(int index) const {
      static const float data[64 + 1][3] = {
        {0.0000000000, 0.0000000000, 0.0000000000}, {0.9999998923, 0.0003078931, 0.0012198694}, {1.9999879340, 0.0046537958, 0.0089553852}, {2.9998345535, 0.0214405108, 0.0261752415}, {3.9990858694, 0.0594703540, 0.0506850977}, {4.9970180023, 0.1232942171, 0.0770580460}, {5.9929196733, 0.2134161267, 0.1034358823}, {6.9860978648, 0.3297733766, 0.1298137186}, {7.9758616801, 0.4722849639, 0.1561915549}, {8.9615225337, 0.6408517645, 0.1825693912}, {9.9423947145, 0.8353565446, 0.2089472275}, {10.9177954640, 1.0556639617, 0.2353250638}, {11.8870461556, 1.3016207441, 0.2617029002}, {12.8494727266, 1.5730556925, 0.2880807365}, {13.8044056782, 1.8697799428, 0.3144585728}, {14.7511803819, 2.1915871378, 0.3408364091}, {15.6891381198, 2.5382534269, 0.3672142454}, {16.6176260811, 2.9095375507, 0.3935920817}, {17.5359984703, 3.3051811902, 0.4199699180}, {18.4436165773, 3.7249089559, 0.4463477543}, {19.3398487945, 4.1684289024, 0.4727255906}, {20.2240715340, 4.6354325755, 0.4991034269}, {21.0956693878, 5.1255950138, 0.5254812633}, {21.9540359498, 5.6385751733, 0.5518590996}, {22.7985742712, 6.1740160196, 0.5782369359}, {23.6286968606, 6.7315449961, 0.6046147722}, {24.4438259508, 7.3107743679, 0.6309926085}, {25.2433944189, 7.9113012217, 0.6573704448}, {26.0268457756, 8.5327076105, 0.6837482811}, {26.7936351031, 9.1745611969, 0.7101261174}, {27.5432291145, 9.8364152266, 0.7365039537}, {28.2751061688, 10.5178093405, 0.7628817900}, {28.9887570281, 11.2182696507, 0.7892596264}, {29.6836850082, 11.9373087444, 0.8156374627}, {30.3594066195, 12.6744263170, 0.8420152990}, {31.0154519334, 13.4291093478, 0.8683931353}, {31.6513645811, 14.2008327341, 0.8947709716}, {32.2667019588, 14.9890597785, 0.9211488079}, {32.8610359510, 15.7932421891, 0.9475266442}, {33.4339529135, 16.6128202723, 0.9739044805}, {33.9850543619, 17.4472238166, 1.0002823168}, {34.5139570164, 18.2958720522, 1.0266601531}, {35.0202928125, 19.1581746928, 1.0530379895}, {35.5037094355, 20.0335320351, 1.0794158258}, {35.9638704483, 20.9213349629, 1.1057936621}, {36.4004556989, 21.8209657377, 1.1321714984}, {36.8131615650, 22.7317982415, 1.1585493347}, {37.2017009540, 23.6531987250, 1.1849271710}, {37.5658034276, 24.5845263972, 1.2113050073}, {37.9052156693, 25.5251334255, 1.2376828436}, {38.2197014609, 26.4743651613, 1.2640606799}, {38.5090420653, 27.4315611892, 1.2904385162}, {38.7730362530, 28.3960552774, 1.3168163526}, {39.0115003064, 29.3671765613, 1.3431941889}, {39.2242682882, 30.3442496582, 1.3695720252}, {39.4111921353, 31.3265946704, 1.3959498615}, {39.5721417980, 32.3135279516, 1.4223276978}, {39.7070053442, 33.3043628311, 1.4487055341}, {39.8156889585, 34.2984100205, 1.4750833704}, {39.8981169749, 35.2949779872, 1.5014612067}, {39.9542164196, 36.2933745674, 1.5277252106}, {39.9852996173, 37.2928693233, 1.5506123912}, {39.9974493868, 38.2927867446, 1.5650501394}, {39.9998937940, 39.2927825739, 1.5703586238}, {40.0000000000, 40.0000000000, 1.5707963268},
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
    SearchRun() {}
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
      deleteTask();
      createTask("SearchRun", SEARCH_RUN_TASK_PRIORITY, SEARCH_RUN_STACK_SIZE);
      //      lg.start();
    }
    void disable() {
      //      lg.end();
      deleteTask();
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
      printf("%s\tRel:(%.1f, %.1f, %.1f)\n", name, sc.position.x, sc.position.y, sc.position.theta * 180 / PI);
    }
  private:
    Position origin;
    std::queue<struct Operation> q;
    bool prev_wall[2];

    void wall_attach() {
#if SEARCH_WALL_ATTACH_ENABLED
      if (tof.getDistance() < 90) {
        portTickType xLastWakeTime = xTaskGetTickCount();
        while (1) {
          const float gain = 0.3f;
          const float satu = 60.0f;
          const float end = 0.5f;
          SpeedController::WheelParameter wp;
          wp.wheel[0] = -std::max(std::min(wd.wall_diff.front[0] * gain, satu), -satu);
          wp.wheel[1] = -std::max(std::min(wd.wall_diff.front[1] * gain, satu), -satu);
          wp.wheel2pole();
          if (fabs(wp.wheel[0]) + fabs(wp.wheel[1]) < end) break;
          sc.set_target(wp.trans, wp.rot);
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        }
        sc.set_target(0, 0);
        printPosition("wall_attach");
        sc.position.x = 0;  //< 直進方向の補正
        sc.position.theta = 0;  //< 回転方向の補正
        bz.play(Buzzer::SHORT);
      }
#endif
    }
    void wall_avoid(const float distance) {
#if SEARCH_WALL_AVOID_ENABLED
      if (fabs(sc.position.theta) < 0.05 * PI) {
        const float gain = 0.0002f; //< ref * gain [mm]
        const float satu = 0.2f;    //< [mm]
        if (ref.side(0) > 60) sc.position.y += std::max(std::min(wd.wall_diff.side[0] * gain, satu), -satu);
        if (ref.side(1) > 60) sc.position.y -= std::max(std::min(wd.wall_diff.side[1] * gain, satu), -satu);
      }
#endif
#if SEARCH_WALL_CUT_ENABLED
      for (int i = 0; i < 2; i++) {
        if (prev_wall[i] && !wd.wall[i] && sc.position.x > 30.0f) {
          const float prev_x = sc.position.x;
          if (distance > 90 - 1)
            sc.position.x = sc.position.x - ((int)sc.position.x) % 90 + 70 - ahead_length;
          printf("WallCut[%d] X_ distance: %.0f, x: %.1f => %.1f\n", i, distance, prev_x, sc.position.x);
        }
        if (!prev_wall[i] && wd.wall[i] && sc.position.x > 30.0f) {
          const float prev_x = sc.position.x;
          if (distance > 90 - 1)
            sc.position.x = sc.position.x - ((int)sc.position.x) % 90 + 60 - ahead_length;
          printf("WallCut[%d] _X distance: %.0f, x: %.1f => %.1f\n", i, distance, prev_x, sc.position.x);
        }
        prev_wall[i] = wd.wall[i];
      }
#endif
    }
    void wall_calib(const float velocity) {
#if SEARCH_WALL_FRONT_ENABLED
      if (wd.wall[2]) {
        float value = tof.getDistance() - (10 + tof.passedTimeMs()) / 1000.0f * velocity;
        float x = sc.position.x;
        if (value > 60 && value < 120) sc.position.x = 90 - value - ahead_length;
        if (sc.position.x > 0.0f) sc.position.x = 0.0f;
        printf("FrontWallCalib: %.2f => %.2f\n", x, sc.position.x);
      }
#endif
    }
    void turn(const float angle) {
      const float speed = 3 * M_PI;
      const float accel = 36 * M_PI;
      const float decel = 24 * M_PI;
      const float back_gain = 2.0f;
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
      const float accel = 3600;
      const float decel = 3600;
      int ms = 0;
      float v_start = sc.actual.trans;
      float T = 1.5f * (v_max - v_start) / accel;
      portTickType xLastWakeTime = xTaskGetTickCount();
      for (int i = 0; i < 2; i++) prev_wall[i] = wd.wall[i];
      while (1) {
        Position cur = sc.position;
        if (v_end >= 1.0f && cur.x > distance - SEARCH_END_REMAIN) break;
        if (v_end < 1.0f && cur.x > distance - 1.0f) break;
        float extra = distance - cur.x - SEARCH_END_REMAIN;
        float velocity_a = v_start + (v_max - v_start) * 6.0f * (-1.0f / 3 * pow(ms / 1000.0f / T, 3) + 1.0f / 2 * pow(ms / 1000.0f / T, 2));
        float velocity_d = sqrt(2 * decel * fabs(extra) + v_end * v_end);
        float velocity = v_max;
        if (velocity > velocity_d) velocity = velocity_d;
        if (ms / 1000.0f < T && velocity > velocity_a) velocity = velocity_a;
        float theta = atan2f(-cur.y, SEARCH_ST_LOOK_AHEAD(velocity)) - cur.theta;
        sc.set_target(velocity, SEARCH_ST_FB_GAIN * theta);
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
        if (tr.getRemain() < SEARCH_END_REMAIN) break;
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        Position dir = tr.getNextDir(sc.position, velocity);
        sc.set_target(velocity, dir.theta);
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
      if (imu.angle.z > 0) {
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
      sc.enable();
      while (1) {
        //** SearchActionがキューされるまで直進で待つ
        {
          S90 tr;
          portTickType xLastWakeTime = xTaskGetTickCount();
          while (q.empty()) {
            vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
            Position cur = sc.position;
            float theta = atan2f(-cur.y, SEARCH_ST_LOOK_AHEAD(velocity)) - cur.theta;
            sc.set_target(velocity, SEARCH_ST_FB_GAIN * theta);
            wall_avoid(0);
          }
        }
        struct Operation operation = q.front();
        enum ACTION action = operation.action;
        int num = operation.num;
        printf("Action: %d %s\n", num, action_string(action));
        printPosition("Start");
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
            straight_x(SEGMENT_WIDTH / 2 * num, velocity, velocity);
            break;
          case TURN_LEFT_90:
            for (int i = 0; i < num; i++) {
              S90 tr(false);
              wall_calib(velocity);
              straight_x(tr.straight - ahead_length, velocity, tr.velocity);
              trace(tr, tr.velocity);
              straight_x(tr.straight + ahead_length, velocity, velocity);
            }
            break;
          case TURN_RIGHT_90:
            for (int i = 0; i < num; i++) {
              S90 tr(true);
              wall_calib(velocity);
              straight_x(tr.straight - ahead_length, velocity, tr.velocity);
              trace(tr, tr.velocity);
              straight_x(tr.straight + ahead_length, velocity, velocity);
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

