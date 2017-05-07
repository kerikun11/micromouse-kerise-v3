#pragma once

#include <Arduino.h>
#include <vector>
#include <queue>
#include "TaskBase.h"
#include "config.h"
#include "logger.h"
#include "as5145.h"
#include "motor.h"
#include "mpu6500.h"
#include "reflector.h"
#include "WallDetector.h"
#include "SpeedController.h"

#define SEARCH_WALL_ATTACH_ENABLED     false
#define SEARCH_WALL_AVOID_ENABLED      false
#define SEARCH_WALL_AVOID_GAIN         0.00002f

#define SEARCH_LOOK_AHEAD   5
#define SEARCH_PROP_GAIN    90

#define SEARCH_RUN_TASK_PRIORITY   3
#define SEARCH_RUN_STACK_SIZE      8192
#define SEARCH_RUN_PERIOD          1000

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
    const float velocity = 100.0f;
    const float straight = 10.0f;
  private:
    bool mirror;
    virtual int size() const {
      return 60;
    }
    virtual Position position(int index) const {
      static const float data[60 + 1][3] = {
        {0.0000000000, 0.0000000000, 0.0000000000}, {0.9999999948, 0.0000141258, 0.0000564894}, {1.9999999666, 0.0002256098, 0.0004509405}, {2.9999995222, 0.0011398968, 0.0015164550}, {3.9999962330, 0.0035901239, 0.0035764853}, {4.9999827250, 0.0087275362, 0.0069401829}, {5.9999386269, 0.0180021210, 0.0118979532}, {6.9998218994, 0.0331431644, 0.0187172793}, {7.9995545553, 0.0561352722, 0.0276388761}, {8.9990026890, 0.0891841370, 0.0388732287}, {9.9979591207, 0.1346894291, 0.0525975657}, {10.9961153446, 0.1952034107, 0.0689533106}, {11.9930386080, 0.2733891065, 0.0880440487}, {12.9881468639, 0.3719815796, 0.1099340369}, {13.9806819155, 0.4937308532, 0.1346472802}, {14.9696928202, 0.6413592371, 0.1621671863}, {15.9540181872, 0.8175056811, 0.1924368044}, {16.9322789605, 1.0246664326, 0.2253596438}, {17.9028778886, 1.2651503980, 0.2608010616}, {18.8640121080, 1.5410168389, 0.2985901990}, {19.8136908997, 1.8540307587, 0.3385224376}, {20.7497671653, 2.2056189697, 0.3803623424}, {21.6699838792, 2.5968285210, 0.4238470469}, {22.5720151378, 3.0283040252, 0.4686900330}, {23.4535338303, 3.5002670955, 0.5145852510}, {24.3122676785, 4.0125124574, 0.5612115208}, {25.1460656902, 4.5644150347, 0.6082371497}, {25.9529590425, 5.1549555645, 0.6553555382}, {26.7311416111, 5.7828463900, 0.7024742720}, {27.4788860145, 6.4466937418, 0.7495930058}, {28.1945324360, 7.1450240349, 0.7967117397}, {28.8764923095, 7.8762871410, 0.8438304735}, {29.5232518445, 8.6388598279, 0.8909492073}, {30.1333753875, 9.4310493642, 0.9380679411}, {30.7055107265, 10.2510958598, 0.9851702039}, {31.2384733041, 11.0971270565, 1.0320459270}, {31.7314266876, 11.9670788754, 1.0783619937}, {32.1839463848, 12.8587377070, 1.1237911723}, {32.5960318401, 13.7697930887, 1.1680189839}, {32.9681086672, 14.6979129640, 1.2107482249}, {33.3010175643, 15.6407981758, 1.2517032404}, {33.5959937794, 16.5962362533, 1.2906338882}, {33.8546287676, 17.5621534746, 1.3273191357}, {34.0788369798, 18.5366445839, 1.3615702381}, {34.2708036724, 19.5180034576, 1.3932334528}, {34.4329325787, 20.5047374223, 1.4221922498}, {34.5677989105, 21.4955726972, 1.4483689875}, {34.6780846709, 22.4894496349, 1.4717260292}, {34.7665336216, 23.4855124683, 1.4922662843}, {34.8358960525, 24.4830912406, 1.5100331663}, {34.8888752312, 25.4816769949, 1.5251099688}, {34.9280892487, 26.4809014803, 1.5376186675}, {34.9560202497, 27.4805071179, 1.5477181665}, {34.9749792386, 28.4803245674, 1.5556020130}, {34.9870712333, 29.4802502921, 1.5614956146}, {34.9941595439, 30.4802243208, 1.5656529975}, {34.9978435224, 31.4802172139, 1.5683531544}, {34.9994319410, 32.4802159134, 1.5698960319}, {34.9999245452, 33.4802157416, 1.5705982181}, {34.9999988313, 34.4802157382, 1.5707883896}, {35.0000000000, 35.0000000068, 1.5707963268},
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
    SearchRun() : TaskBase("SearchRun", SEARCH_RUN_TASK_PRIORITY, SEARCH_RUN_STACK_SIZE) {
      xLastWakeTime = xTaskGetTickCount();
    }
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
    //    void setPosition(Position pos = Position(SEGMENT_WIDTH / 2, WALL_THICKNESS / 2 + MACHINE_TAIL_LENGTH, M_PI / 2)) {
    void setPosition(Position pos = Position(0, 0, 0)) {
      origin = pos;
      sc.getPosition() = pos;
    }
    void fixPosition(Position pos) {
      sc.getPosition() -= pos;
    }
  private:
    portTickType xLastWakeTime;
    Position origin;
    std::queue<struct Operation> q;

    void wall_avoid() {
#if SEARCH_WALL_AVOID_ENABLED
      const float gain = SEARCH_WALL_AVOID_GAIN;
      if (wd.wall().side[0]) {
        fixPosition(Position(0, wd.wall_difference().side[0] * gain * sc.actual.trans, 0).rotate(origin.theta));
      }
      if (wd.wall().side[1]) {
        fixPosition(Position(0, -wd.wall_difference().side[1] * gain * sc.actual.trans, 0).rotate(origin.theta));
      }
#endif
    }
    void wall_attach() {
#if SEARCH_WALL_ATTACH_ENABLED
      if (wd.wall().front[0] && wd.wall().front[1]) {
        while (1) {
          float trans = (wd.wall_difference().front[0] + wd.wall_difference().front[1]) * 100;
          float rot = (wd.wall_difference().front[1] - wd.wall_difference().front[0]) * 50;
          if (fabs(trans) < 5.0f && fabs(rot) < 0.5f) break;
          sc.set_target(trans, rot);
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        }
        sc.set_target(0, 0);
        printPosition("1");
        //        fixPosition(Position(getRelativePosition().x, 0, getRelativePosition().theta).rotate(origin.theta));
        fixPosition(Position(getRelativePosition().x, 0, 0).rotate(origin.theta));
        printPosition("2");
        bz.play(Buzzer::SELECT);
        delay(1000);
      }
#endif
    }
    void turn(const float angle) {
      const float speed = 2 * M_PI;
      const float accel = 24 * M_PI;
      const float back_gain = 160.0f;
      int ms = 0;
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
        if (fabs(sc.actual.rot) < 0.1) break;
        float extra = angle - getRelativePosition().theta;
        float target_speed = sqrt(2 * accel * fabs(extra));
        float delta = getRelativePosition().x * cos(-getRelativePosition().theta) - getRelativePosition().y * sin(-getRelativePosition().theta);
        target_speed = (target_speed > speed) ? speed : target_speed;
        if (extra > 0) {
          sc.set_target(-delta * back_gain, target_speed);
        } else {
          sc.set_target(-delta * back_gain, -target_speed);
        }
      }
      updateOrigin(Position(0, 0, angle));
      printPosition("Turn End");
    }
    void straight_x(const float distance, const float v_max, const float v_end, bool avoid) {
      const float accel = 900;
      const float decel = 900;
      int ms = 0;
      float v_start = sc.actual.trans;
      float T = 1.5f * (v_max - v_start) / accel;
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
        if (avoid) wall_avoid();
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        ms++;
      }
      sc.set_target(v_end, 0);
      updateOrigin(Position(distance, 0, 0));
      printPosition("Straight End");
    }
    template<class C>
    void trace(C tr, const float velocity) {
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
    virtual void task() {
      const float velocity = 100;
      const float ahead_length = 5.0f;
      sc.enable();
      while (1) {
        while (q.empty()) {
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
          S90 tr;
          Position cur = getRelativePosition();
          const float decel = 1200;
          float extra = tr.straight - ahead_length - cur.x - SEARCH_LOOK_AHEAD;
          float v = sqrt(2 * decel * fabs(extra));
          if (v > velocity) v = velocity;
          if (extra < 0) v = -v;
          float theta = atan2f(-cur.y, SEARCH_LOOK_AHEAD) - cur.theta;
          sc.set_target(v, SEARCH_PROP_GAIN * theta);
          wall_avoid();
        }
        struct Operation operation = q.front();
        enum ACTION action = operation.action;
        int num = operation.num;
        printf("%s: %d\n", action_string(action), num);
        printPosition("Start");
        switch (action) {
          case START_STEP:
            setPosition();
            straight_x(SEGMENT_WIDTH - MACHINE_TAIL_LENGTH - WALL_THICKNESS / 2 + ahead_length, velocity, velocity, true);
            break;
          case START_INIT:
            straight_x(SEGMENT_WIDTH / 2 - ahead_length, velocity, 0, true);
            wall_attach();
            turn(M_PI / 2);
            wall_attach();
            turn(M_PI / 2);
            for (int i = 0; i < 160; i++) {
              sc.set_target(-i, -getRelativePosition().theta * 100.0f);
              delay(1);
            }
            delay(200);
            sc.disable();
            mt.drive(-60, -60);
            delay(400);
            mt.free();
            while (q.size()) {
              q.pop();
            }
            while (1) {
              delay(1000);
            }
          case GO_STRAIGHT:
            straight_x(SEGMENT_WIDTH * num, velocity, velocity, true);
            break;
          case GO_HALF:
            straight_x(SEGMENT_WIDTH / 2 * num, velocity, velocity, true);
            break;
          case TURN_LEFT_90:
            for (int i = 0; i < num; i++) {
              S90 tr(false);
              straight_x(tr.straight - ahead_length, velocity, tr.velocity, true);
              trace(tr, tr.velocity);
              straight_x(tr.straight + ahead_length, tr.velocity, velocity, true);
            }
            break;
          case TURN_RIGHT_90:
            for (int i = 0; i < num; i++) {
              S90 tr(true);
              straight_x(tr.straight - ahead_length, velocity, tr.velocity, true);
              trace(tr, tr.velocity);
              straight_x(tr.straight + ahead_length, tr.velocity, velocity, true);
            }
            break;
          case TURN_BACK:
            straight_x(SEGMENT_WIDTH / 2 - ahead_length, velocity, 0, true);
            if (mpu.angle.z > 0) {
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
            straight_x(SEGMENT_WIDTH / 2 + ahead_length, velocity, velocity, true);
            break;
          case RETURN:
            if (mpu.angle.z > 0) {
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

