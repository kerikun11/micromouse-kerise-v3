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
#define SEARCH_WALL_AVOID_ENABLED   false
#define SEARCH_WALL_AVOID_GAIN      0.0000005f

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
    const float velocity = 270.0f;
    const float straight = 5.0f;
    //    const float straight = 10.0f;
  private:
    bool mirror;
    virtual int size() const {
      return 69;
      //      return 60;
    }
    virtual Position position(int index) const {
      static const float data[69 + 1][3] = {
        {0.0000000000, 0.0000000000, 0.0000000000}, {0.9999999962, 0.0000094642, 0.0000378499}, {1.9999999773, 0.0001511747, 0.0003022989}, {2.9999997995, 0.0007644404, 0.0010174506}, {3.9999983085, 0.0024095148, 0.0024024453}, {4.9999921290, 0.0058630426, 0.0046690468}, {5.9999722739, 0.0121091045, 0.0080193101}, {6.9999189950, 0.0223262955, 0.0126433563}, {7.9997965880, 0.0378780606, 0.0187172793}, {8.9995430023, 0.0602954548, 0.0264012060}, {9.9990595129, 0.0912578826, 0.0358375335}, {10.9982003001, 0.1325784102, 0.0471493602}, {11.9967555602, 0.1861785380, 0.0604391311}, {12.9944384799, 0.2540640374, 0.0757875088}, {13.9908713857, 0.3383049548, 0.0932524857}, {14.9855679304, 0.4410028843, 0.1128687454}, {15.9779221178, 0.5642634348, 0.1346472802}, {16.9671963943, 0.7101708672, 0.1585752707}, {17.9525117713, 0.8807498286, 0.1846162266}, {18.9328440736, 1.0779370372, 0.2127103886}, {19.9070217554, 1.3035522259, 0.2427753866}, {20.8737321334, 1.5592597075, 0.2747071467}, {21.8315285645, 1.8465435433, 0.3083810374}, {22.7788447518, 2.1666798452, 0.3436532420}, {23.7140200547, 2.5207072335, 0.3803623424}, {24.6353182501, 2.9094107281, 0.4183310968}, {25.5409593801, 3.3333037881, 0.4573683914}, {26.4291594281, 3.7926172309, 0.4972713452}, {27.2981564990, 4.2872958982, 0.5378275442}, {28.1462563341, 4.8170001824, 0.5788173804}, {28.9718709563, 5.3811090832, 0.6200164878}, {29.7735329825, 5.9787681433, 0.6612453799}, {30.5498799234, 6.6089615927, 0.7024742720}, {31.2995923145, 7.2706183668, 0.7437031641}, {32.0213959591, 7.9626139265, 0.7849320562}, {32.7140640938, 8.6837721694, 0.8261609483}, {33.3764194730, 9.4328674290, 0.8673898404}, {34.0073363702, 10.2086265572, 0.9086187325}, {34.6057424913, 11.0097310888, 0.9498476246}, {35.1706259794, 11.8348160331, 0.9910488666}, {35.7011163327, 12.6824252427, 1.0320459270}, {36.1965914714, 13.5509669668, 1.0726143799}, {36.6567063307, 14.4387521442, 1.1125344827}, {37.0813985363, 15.3440203128, 1.1515936324}, {37.4708935550, 16.2649825872, 1.1895887073}, {37.8256971959, 17.1998636497, 1.2263283039}, {38.1465883403, 18.1469251523, 1.2616348444}, {38.4346030618, 19.1045012487, 1.2953465326}, {38.6910084063, 20.0710266805, 1.3273191357}, {38.9172851174, 21.0450509275, 1.3574275722}, {39.1150972597, 22.0252572879, 1.3855672900}, {39.2862561207, 23.0104719004, 1.4116554172}, {39.4326983093, 23.9996666706, 1.4356316752}, {39.5564497952, 24.9919603265, 1.4574590420}, {39.6595885621, 25.9866109199, 1.4771241604}, {39.7442228222, 26.9830098484, 1.4946374848}, {39.8124565654, 27.9806698637, 1.5100331663}, {39.8663563879, 28.9792085621, 1.5233686764}, {39.9079325713, 29.9783382261, 1.5347241740}, {39.9391098451, 30.9778488363, 1.5442016230}, {39.9617006069, 31.9775909539, 1.5519236692}, {39.9773899210, 32.9774661280, 1.5580322898}, {39.9877127586, 33.9774122382, 1.5626872297}, {39.9940343177, 34.9773916692, 1.5660642423}, {39.9975393246, 35.9773852398, 1.5683531544}, {39.9992162983, 36.9773838523, 1.5697557753}, {39.9998455949, 37.9773835975, 1.5704836745}, {39.9999934554, 38.9773835831, 1.5707558520}, {40.0000038070, 39.9773835832, 1.5707963264}, {40.0000000000, 40.0000000078, 1.5707963268},
        //        {0.0000000000, 0.0000000000, 0.0000000000}, {0.9999999963, 0.0000205486, 0.0000821611}, {1.9999999452, 0.0003279763, 0.0006549044}, {2.9999989469, 0.0016537203, 0.0021969514}, {3.9999923373, 0.0051971078, 0.0051635894}, {4.9999639492, 0.0125964594, 0.0099756641}, {5.9998735978, 0.0258890878, 0.0170094049}, {6.9996369863, 0.0474618412, 0.0265873293}, {7.9991014226, 0.0799918543, 0.0389704454}, {8.9980149776, 0.1263799401, 0.0543519415}, {9.9959955351, 0.1896759060, 0.0728525157}, {10.9924984539, 0.2729990108, 0.0945174578}, {11.9867903213, 0.3794528500, 0.1193155543}, {12.9779292868, 0.5120389408, 0.1471398431}, {13.9647572959, 0.6735688748, 0.1778101995}, {14.9459068561, 0.8665804950, 0.2110776906}, {15.9198222895, 1.0932588517, 0.2466305943}, {16.8848000427, 1.3553679966, 0.2841019351}, {17.8390408392, 1.6541948871, 0.3230783564}, {18.7807193337, 1.9905105696, 0.3631101141}, {19.7080582101, 2.3645494607, 0.4037219499}, {20.6194065410, 2.7760173968, 0.4444691820}, {21.5132366320, 3.2242688996, 0.4852169296}, {22.3880645951, 3.7085598067, 0.5259646771}, {23.2424380765, 4.2280861175, 0.5667124247}, {24.0749387046, 4.7819853510, 0.6074601723}, {24.8841843828, 5.3693379373, 0.6482079198}, {25.6688316658, 5.9891688004, 0.6889556674}, {26.4275779002, 6.6404489115, 0.7297034149}, {27.1591634713, 7.3220970630, 0.7704511625}, {27.8623738265, 8.0329816068, 0.8111989101}, {28.5360415371, 8.7719223737, 0.8519466576}, {29.1790482151, 9.5376926133, 0.8926944052}, {29.7903263670, 10.3290210244, 0.9334421527}, {30.3688611912, 11.1445938990, 0.9741899003}, {30.9136922188, 11.9830572433, 1.0149376478}, {31.4239149646, 12.8430191071, 1.0556853954}, {31.8986823684, 13.7230518017, 1.0964331430}, {32.3372062557, 14.6216943592, 1.1371808905}, {32.7387576878, 15.5374553421, 1.1779211079}, {33.1027560682, 16.4687815421, 1.2184258693}, {33.4290408514, 17.4139831801, 1.2582221083}, {33.7179903006, 18.3712626373, 1.2968402605}, {33.9705156772, 19.3387939028, 1.3338392435}, {34.1880355674, 20.3147975651, 1.3688171206}, {34.3724322298, 21.2976043688, 1.4014208185}, {34.5259918206, 22.2857059665, 1.4313546632}, {34.6513331378, 23.2777889378, 1.4583875282}, {34.7513267092, 24.2727529420, 1.4823584225}, {34.8290098859, 25.2697129189, 1.5031803829}, {34.8874987874, 26.2679879370, 1.5208425762}, {34.9299023666, 27.2670796617, 1.5354105612}, {34.9592382638, 28.2666436299, 1.5470247040}, {34.9783547687, 29.2664576275, 1.5558967840}, {34.9898578484, 30.2663897401, 1.5623048740}, {34.9960467107, 31.2663698370, 1.5665866175}, {34.9988567925, 32.2663656122, 1.5691310656}, {34.9998129935, 33.2663650971, 1.5703692709}, {34.9999923534, 34.2663650745, 1.5707638666}, {35.0000000000, 35.0000000041, 1.5707963268},
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
      printf("%s\tRel:(%06.1f, %06.1f, %06.3f)\n", name, sc.position.x, sc.position.y, sc.position.theta);
    }
  private:
    Position origin;
    std::queue<struct Operation> q;
    bool prev_wall[2];

    void wall_attach() {
#if SEARCH_WALL_ATTACH_ENABLED
      if (wd.wall[2]) {
        portTickType xLastWakeTime = xTaskGetTickCount();
        int cnt = 0;
        while (1) {
          float trans = -(wd.wall_diff.front[0] + wd.wall_diff.front[1]) * 0.5f; //< 0.5
          float rot = (wd.wall_diff.front[0] - wd.wall_diff.front[1]) * 0.01f; //< 0.01
          const float trans_sat = 80;
          const float rot_sat = 0.5 * PI;
          if (trans > trans_sat) trans = trans_sat;
          else if (trans < -trans_sat)trans = -trans_sat;
          if (rot > rot_sat) rot = rot_sat;
          else if (rot < -rot_sat)rot = -rot_sat;
          if (fabs(trans) < 0.001f && fabs(rot) < 0.005f) break;
          sc.set_target(trans, rot);
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
          if (cnt++ % 100 == 0) printf("trans: %f, rot:%f\n", rot, trans);
        }
        sc.set_target(0, 0);
        printPosition("wall_attach");
        sc.position.x = 0;  //< 直進方向の補正
        sc.position.theta = 0;  //< 直進方向の補正
        bz.play(Buzzer::CONFIRM);
      }
#endif
    }
    void wall_avoid(const float distance) {
#if SEARCH_WALL_AVOID_ENABLED
      const float gain = SEARCH_WALL_AVOID_GAIN;
      if (wd.getWall(0)) {
        float x = -wd.wall_diff.side[0] * gain * sc.actual.trans;
        fixPosition(Position(0, x, 0).rotate(origin.theta));
      }
      if (wd.getWall(1)) {
        fixPosition(Position(0, wd.wall_diff.side[1] * gain * sc.actual.trans, 0).rotate(origin.theta));
      }
#endif
#if SEARCH_WALL_CUT_ENABLED
      for (int i = 0; i < 2; i++) {
        if (prev_wall[i] && !wd.wall[i]) {
          printf("WallCut[%d]X_: dist => %.0f, x => %.2f", i, distance, sc.position.x);
          //          bz.play(Buzzer::CANCEL);
          if (abs(distance - 45) < 1.0f)
            sc.position.x = 45 - 20;
          if (abs(distance - 90) < 1.0f)
            sc.position.x = 90 - 20;
          if (distance > 90 + 1)
            sc.position.x = sc.position.x - ((int)sc.position.x) % 90 + 70;
        }
        if (!prev_wall[i] && wd.wall[i]) {
          printf("WallCut[%d]_X: dist => %.0f, x => %.2f", i, distance, sc.position.x);
          //          bz.play(Buzzer::CONFIRM);
          if (abs(distance - 45) < 1.0f)
            sc.position.x = 45 - 20;
          if (abs(distance - 90) < 1.0f)
            sc.position.x = 90 - 20;
          if (distance > 90 + 1)
            sc.position.x = sc.position.x - ((int)sc.position.x) % 90 + 70;
        }
        prev_wall[i] = wd.wall[i];
      }
#endif
#if 0
      if (tof.passedTimeMs() == 0) {
        if (tof.getDistance() > 90 && tof.getDistance() < 150) {
          float value = tof.getDistance() - 10.0f / 1000.0f * sc.actual.trans;
          if (abs(distance - 45) < 1.0f) {
            sc.position.x = 135 - value;
            bz.play(Buzzer::SHORT);
          } else if (abs(distance - 90) < 1.0f) {
            sc.position.x = 180 - value;
            bz.play(Buzzer::SHORT);
          }
        }
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
        if (fabs(sc.actual.rot) < 0.1 && abs(extra) < 0.1) break;
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
    void straight_x(const float distance, const float v_max, const float v_end, bool avoid) {
      const float accel = 3000;
      const float decel = 2000;
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
        float theta = atan2f(-cur.y, SEARCH_LOOK_AHEAD + velocity / 50) - cur.theta;
        sc.set_target(velocity, SEARCH_PROP_GAIN * theta);
        if (avoid) wall_avoid(distance);
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
    void wall_calib(const float velocity) {
#if 0
      if (wd.wall[2]) {
        float value = tof.getDistance() - (10.0f + tof.passedTimeMs()) / 1000.0f * velocity;
        fixPosition(Position(sc.position.x - (90 - value), 0, 0).rotate(origin.theta));
        bz.play(Buzzer::SHORT);
        printf("FrontWallCalib: %f", value);
      }
#endif
    }
    virtual void task() {
      const float velocity = 300;
      const float v_max = 360;
      const float ahead_length = 0.0f;
      sc.enable();
      while (1) {
        {
          S90 tr;
          portTickType xLastWakeTime = xTaskGetTickCount();
          while (q.empty()) {
            vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
            Position cur = sc.position;
            const float decel = 2000;
            float extra = tr.straight - ahead_length - cur.x - SEARCH_LOOK_AHEAD;
            float v = sqrt(2 * decel * fabs(extra));
            if (v > velocity) v = velocity;
            if (extra < 0) v = -v;
            float theta = atan2f(-cur.y, SEARCH_LOOK_AHEAD) - cur.theta;
            sc.set_target(v, SEARCH_PROP_GAIN * theta);
            //            wall_avoid(0);
          }
        }
        struct Operation operation = q.front();
        enum ACTION action = operation.action;
        int num = operation.num;
        printf("%s: %d\n", action_string(action), num);
        printPosition("Start");
        switch (action) {
          case START_STEP:
            sc.position.reset();
            straight_x(SEGMENT_WIDTH - MACHINE_TAIL_LENGTH - WALL_THICKNESS / 2 + ahead_length, velocity, velocity, true);
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

