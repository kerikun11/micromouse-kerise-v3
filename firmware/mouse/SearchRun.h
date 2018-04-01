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
#define SEARCH_CURVE_FB_GAIN        6.0f

#define ahead_length                0

#define SEARCH_RUN_TASK_PRIORITY    3
#define SEARCH_RUN_STACK_SIZE       8192
#define SEARCH_RUN_PERIOD           1000

#define SEARCH_RUN_VELOCITY         300.0f
#define SEARCH_RUN_V_CURVE          300.0f
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
    const float straight = 1.0f;
  private:
    bool mirror;
    virtual int size() const {
      return 71;
    }
    virtual Position position(int index) const {
      static const float data[71 + 1][3] = {
        {0.0000000000, 0.0000000000, 0.0000000000}, {0.9999999387, 0.0002320892, 0.0009210643}, {1.9999929768, 0.0035427338, 0.0068636320}, {2.9998999853, 0.0165885973, 0.0205711497}, {3.9994180863, 0.0470451367, 0.0412644139}, {4.9979858467, 0.1000971410, 0.0650681204}, {5.9949944819, 0.1770771129, 0.0890479716}, {6.9898706249, 0.2779407850, 0.1130278228}, {7.9820422894, 0.4026301268, 0.1370076740}, {8.9709390025, 0.5510734581, 0.1609875252}, {9.9559922426, 0.7231854561, 0.1849673763}, {10.9366352983, 0.9188671547, 0.2089472275}, {11.9123042336, 1.1380060329, 0.2329270787}, {12.8824382659, 1.3804760569, 0.2569069299}, {13.8464797654, 1.6461377458, 0.2808867811}, {14.8038742587, 1.9348383919, 0.3048666323}, {15.7540711996, 2.2464120630, 0.3288464835}, {16.6965240673, 2.5806796021, 0.3528263347}, {17.6306908433, 2.9374487565, 0.3768061859}, {18.5560345846, 3.3165143761, 0.4007860371}, {19.4720234235, 3.7176583931, 0.4247658883}, {20.3781305675, 4.1406502108, 0.4487457394}, {21.2738348990, 4.5852467161, 0.4727255906}, {22.1586213057, 5.0511922800, 0.4967054418}, {23.0319808899, 5.5382188921, 0.5206852930}, {23.8934116433, 6.0460465273, 0.5446651442}, {24.7424184466, 6.5743830330, 0.5686449954}, {25.5785130697, 7.1229246804, 0.5926248466}, {26.4012146077, 7.6913562000, 0.6166046978}, {27.2100499857, 8.2793507820, 0.6405845490}, {28.0045539562, 8.8865701978, 0.6645644002}, {28.7842698194, 9.5126653341, 0.6885442514}, {29.5487494315, 10.1572759914, 0.7125241025}, {30.2975532051, 10.8200315766, 0.7365039537}, {31.0302504080, 11.5005511778, 0.7604838049}, {31.7464197557, 12.1984435646, 0.7844636561}, {32.4456492804, 12.9133072859, 0.8084435073}, {33.1275370307, 13.6447313482, 0.8324233585}, {33.7916910951, 14.3922949552, 0.8564032097}, {34.4377296024, 15.1555682980, 0.8803830609}, {35.0652809132, 15.9341126917, 0.9043629121}, {35.6739842193, 16.7274805755, 0.9283427633}, {36.2634893600, 17.5352155804, 0.9523226145}, {36.8334574419, 18.3568533157, 0.9763024656}, {37.3835608816, 19.1919210944, 1.0002823168}, {37.9134834050, 20.0399387646, 1.0242621680}, {38.4229201592, 20.9004189308, 1.0482420192}, {38.9115782530, 21.7728669536, 1.0722218704}, {39.3791765804, 22.6567809889, 1.0962017216}, {39.8254463149, 23.5516528304, 1.1201815728}, {40.2501309677, 24.4569676833, 1.1441614240}, {40.6529863877, 25.3722049665, 1.1681412752}, {41.0337808175, 26.2968386421, 1.1921211264}, {41.3922953245, 27.2303372151, 1.2161009776}, {41.7283236746, 28.1721637498, 1.2400808287}, {42.0416726709, 29.1217767087, 1.2640606799}, {42.3321622184, 30.0786298454, 1.2880405311}, {42.5996253243, 31.0421728997, 1.3120203823}, {42.8439081201, 32.0118520578, 1.3360002335}, {43.0648701505, 32.9871099524, 1.3599800847}, {43.2623843192, 33.9673856644, 1.3839599359}, {43.4363370601, 34.9521154080, 1.4079397871}, {43.5866283939, 35.9407329061, 1.4319196383}, {43.7131719281, 36.9326697047, 1.4558994895}, {43.8158948609, 37.9273556042, 1.4798793407}, {43.8947381121, 38.9242186583, 1.5038591918}, {43.9496552431, 39.9226858593, 1.5277252106}, {43.9817399481, 40.9221522999, 1.5488289099}, {43.9958782666, 41.9220437282, 1.5631563448}, {43.9996733307, 42.9220347589, 1.5696470801}, {43.9999858797, 43.9220346559, 1.5707958803}, {44.0000000000, 43.9999999920, 1.5707963268},
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
            sc.position.x = sc.position.x - ((int)sc.position.x) % 90 + 76 - ahead_length;
          printf("WallCut[%d] X_ distance: %.0f, x: %.1f => %.1f\n", i, distance, prev_x, sc.position.x);
        }
        if (!prev_wall[i] && wd.wall[i] && sc.position.x > 30.0f) {
          const float prev_x = sc.position.x;
          if (distance > 90 - 1)
            sc.position.x = sc.position.x - ((int)sc.position.x) % 90 + 64 - ahead_length;
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
      // キャリブレーション
      bz.play(Buzzer::CONFIRM);
      imu.calibration();
      bz.play(Buzzer::CANCEL);
      // スタート
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

