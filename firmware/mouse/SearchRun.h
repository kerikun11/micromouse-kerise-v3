#pragma once

#include <Arduino.h>
#include <vector>
#include <queue>
#include "TaskBase.h"
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "imu.h"
#include "reflector.h"
#include "Logger.h"
#include "WallDetector.h"
#include "SpeedController.h"

#define SEARCH_WALL_ATTACH_ENABLED  true
#define SEARCH_WALL_CUT_ENABLED     true
#define SEARCH_WALL_FRONT_ENABLED   true
#define SEARCH_WALL_AVOID_ENABLED   true

#define SEARCH_LOOK_AHEAD   9
#define SEARCH_PROP_GAIN    20
#define ahead_length        7

#define SEARCH_RUN_TASK_PRIORITY    3
#define SEARCH_RUN_STACK_SIZE       8192
#define SEARCH_RUN_PERIOD           1000

#define SEARCH_RUN_VELOCITY         240.0f
#define SEARCH_RUN_V_CURVE          240.0f
#define SEARCH_RUN_V_MAX            900.0f

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
      return 64;
    }
    virtual Position position(int index) const {
      static const float data[64 + 1][3] = {
        //        {0.0000000000, 0.0000000000, 0.0000000000}, {0.9999999962, 0.0000094642, 0.0000378499}, {1.9999999773, 0.0001511747, 0.0003022989}, {2.9999997995, 0.0007644404, 0.0010174506}, {3.9999983085, 0.0024095148, 0.0024024453}, {4.9999921290, 0.0058630426, 0.0046690468}, {5.9999722739, 0.0121091045, 0.0080193101}, {6.9999189950, 0.0223262955, 0.0126433563}, {7.9997965880, 0.0378780606, 0.0187172793}, {8.9995430023, 0.0602954548, 0.0264012060}, {9.9990595129, 0.0912578826, 0.0358375335}, {10.9982003001, 0.1325784102, 0.0471493602}, {11.9967555602, 0.1861785380, 0.0604391311}, {12.9944384799, 0.2540640374, 0.0757875088}, {13.9908713857, 0.3383049548, 0.0932524857}, {14.9855679304, 0.4410028843, 0.1128687454}, {15.9779221178, 0.5642634348, 0.1346472802}, {16.9671963943, 0.7101708672, 0.1585752707}, {17.9525117713, 0.8807498286, 0.1846162266}, {18.9328440736, 1.0779370372, 0.2127103886}, {19.9070217554, 1.3035522259, 0.2427753866}, {20.8737321334, 1.5592597075, 0.2747071467}, {21.8315285645, 1.8465435433, 0.3083810374}, {22.7788447518, 2.1666798452, 0.3436532420}, {23.7140200547, 2.5207072335, 0.3803623424}, {24.6353182501, 2.9094107281, 0.4183310968}, {25.5409593801, 3.3333037881, 0.4573683914}, {26.4291594281, 3.7926172309, 0.4972713452}, {27.2981564990, 4.2872958982, 0.5378275442}, {28.1462563341, 4.8170001824, 0.5788173804}, {28.9718709563, 5.3811090832, 0.6200164878}, {29.7735329825, 5.9787681433, 0.6612453799}, {30.5498799234, 6.6089615927, 0.7024742720}, {31.2995923145, 7.2706183668, 0.7437031641}, {32.0213959591, 7.9626139265, 0.7849320562}, {32.7140640938, 8.6837721694, 0.8261609483}, {33.3764194730, 9.4328674290, 0.8673898404}, {34.0073363702, 10.2086265572, 0.9086187325}, {34.6057424913, 11.0097310888, 0.9498476246}, {35.1706259794, 11.8348160331, 0.9910488666}, {35.7011163327, 12.6824252427, 1.0320459270}, {36.1965914714, 13.5509669668, 1.0726143799}, {36.6567063307, 14.4387521442, 1.1125344827}, {37.0813985363, 15.3440203128, 1.1515936324}, {37.4708935550, 16.2649825872, 1.1895887073}, {37.8256971959, 17.1998636497, 1.2263283039}, {38.1465883403, 18.1469251523, 1.2616348444}, {38.4346030618, 19.1045012487, 1.2953465326}, {38.6910084063, 20.0710266805, 1.3273191357}, {38.9172851174, 21.0450509275, 1.3574275722}, {39.1150972597, 22.0252572879, 1.3855672900}, {39.2862561207, 23.0104719004, 1.4116554172}, {39.4326983093, 23.9996666706, 1.4356316752}, {39.5564497952, 24.9919603265, 1.4574590420}, {39.6595885621, 25.9866109199, 1.4771241604}, {39.7442228222, 26.9830098484, 1.4946374848}, {39.8124565654, 27.9806698637, 1.5100331663}, {39.8663563879, 28.9792085621, 1.5233686764}, {39.9079325713, 29.9783382261, 1.5347241740}, {39.9391098451, 30.9778488363, 1.5442016230}, {39.9617006069, 31.9775909539, 1.5519236692}, {39.9773899210, 32.9774661280, 1.5580322898}, {39.9877127586, 33.9774122382, 1.5626872297}, {39.9940343177, 34.9773916692, 1.5660642423}, {39.9975393246, 35.9773852398, 1.5683531544}, {39.9992162983, 36.9773838523, 1.5697557753}, {39.9998455949, 37.9773835975, 1.5704836745}, {39.9999934554, 38.9773835831, 1.5707558520}, {40.0000038070, 39.9773835832, 1.5707963264}, {40.0000000000, 40.0000000000, 1.5707963268},
        //        {0.0000000000, 0.0000000000, 0.0000000000}, {0.9999999926, 0.0000773152, 0.0003086046}, {1.9999991533, 0.0012208386, 0.0024200803}, {2.9999862627, 0.0060454113, 0.0079003866}, {3.9999042317, 0.0185252422, 0.0178727920}, {4.9995836606, 0.0434702426, 0.0328702678}, {5.9986670223, 0.0858893497, 0.0527654514}, {6.9965648523, 0.1503232120, 0.0767873824}, {7.9924843397, 0.2402355913, 0.1036228134}, {8.9855474634, 0.3575387753, 0.1316060125}, {9.9749291744, 0.5026540310, 0.1596625310}, {10.9598504769, 0.6754671406, 0.1877190495}, {11.9395361497, 0.8758420991, 0.2157755680}, {12.9132153208, 1.1036211226, 0.2438320865}, {13.8801215147, 1.3586249512, 0.2718886050}, {14.8394936084, 1.6406529254, 0.2999451235}, {15.7905762684, 1.9494830264, 0.3280016420}, {16.7326210354, 2.2848721543, 0.3560581605}, {17.6648866079, 2.6465562491, 0.3841146790}, {18.5866390333, 3.0342507041, 0.4121711975}, {19.4971527336, 3.4476504135, 0.4402277160}, {20.3957108952, 3.8864299293, 0.4682842345}, {21.2816064643, 4.3502438310, 0.4963407530}, {22.1541422290, 4.8387270063, 0.5243972715}, {23.0126312621, 5.3514950905, 0.5524537900}, {23.8563977390, 5.8881444796, 0.5805103085}, {24.6847774969, 6.4482527333, 0.6085668270}, {25.4971187237, 7.0313788618, 0.6366233455}, {26.2927819795, 7.6370639222, 0.6646798639}, {27.0711409235, 8.2648313203, 0.6927363824}, {27.8315827460, 8.9141868660, 0.7207929009}, {28.5735089838, 9.5846194128, 0.7488494194}, {29.2963358274, 10.2756011096, 0.7769059379}, {29.9994942217, 10.9865882082, 0.8049624564}, {30.6824306630, 11.7170212061, 0.8330189749}, {31.3446074863, 12.4663250549, 0.8610754934}, {31.9855036265, 13.2339099076, 0.8891320119}, {32.6046147173, 14.0191714937, 0.9171885304}, {33.2014533312, 14.8214919213, 0.9452450489}, {33.7755496421, 15.6402397153, 0.9733015674}, {34.3264517308, 16.4747703355, 1.0013580859}, {34.8537261177, 17.3244267715, 1.0294146044}, {35.3569577789, 18.1885402664, 1.0574711229}, {35.8357505544, 19.0664308867, 1.0855276414}, {36.2897275025, 19.9574075649, 1.1135841599}, {36.7185313255, 20.8607689543, 1.1416406784}, {37.1218246124, 21.7758038096, 1.1696971969}, {37.4992898733, 22.7017920395, 1.1977537154}, {37.8506299754, 23.6380049772, 1.2258102338}, {38.1755683152, 24.5837055712, 1.2538667523}, {38.4738491902, 25.5381493897, 1.2819232708}, {38.7452378736, 26.5005850279, 1.3099797893}, {38.9895206838, 27.4702551690, 1.3380363078}, {39.2065053397, 28.4463966696, 1.3660928263}, {39.3960210567, 29.4282409544, 1.3941493448}, {39.5579186838, 30.4150152769, 1.4222058633}, {39.6919981719, 31.4059533883, 1.4502623811}, {39.7983801811, 32.4002464178, 1.4779980248}, {39.8779145035, 33.3970504192, 1.5039081843}, {39.9331747718, 34.3955011822, 1.5264314840}, {39.9681150277, 35.3948770171, 1.5444516085}, {39.9875386677, 36.3946813128, 1.5574439951}, {39.9964559987, 37.3946388123, 1.5655447656}, {39.9994186253, 38.3946337576, 1.5695328390}, {39.9999182025, 39.3946335725, 1.5707275750}, {40.0000000000, 40.0000000000, 1.5707963268},
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
          SpeedController::WheelParameter wp;
          const float gain = 0.3f;
          const float satu = 100.0f;
          wp.wheel[0] = -std::max(std::min(wd.wall_diff.front[0] * gain, satu), -satu);
          wp.wheel[1] = -std::max(std::min(wd.wall_diff.front[1] * gain, satu), -satu);
          wp.wheel2pole();
          if (fabs(wp.wheel[0]) + fabs(wp.wheel[1]) < 0.5f) break;
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
      if (fabs(sc.position.theta) < 0.05 * PI) {
        const float gain = 0.0002f;
        const float satu = 0.1f;
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
        //        bz.play(Buzzer::SHORT);
        if (sc.position.x > -2.0f) sc.position.x = -2.0f;
        printf("FrontWallCalib: %.2f => %.2f\n", x, sc.position.x);
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
      const float accel = 2400;
      const float decel = 1200;
      int ms = 0;
      float v_start = sc.actual.trans;
      float T = 1.5f * (v_max - v_start) / accel;
      portTickType xLastWakeTime = xTaskGetTickCount();
      for (int i = 0; i < 2; i++) prev_wall[i] = wd.wall[i];
      while (1) {
        Position cur = sc.position;
        if (v_end >= 1.0f && cur.x > distance - SEARCH_LOOK_AHEAD) break;
        if (v_end < 1.0f && cur.x > distance - 1.0f) break;
        float extra = distance - cur.x - SEARCH_LOOK_AHEAD;
        float velocity_a = v_start + (v_max - v_start) * 6.0f * (-1.0f / 3 * pow(ms / 1000.0f / T, 3) + 1.0f / 2 * pow(ms / 1000.0f / T, 2));
        float velocity_d = sqrt(2 * decel * fabs(extra) + v_end * v_end);
        float velocity = v_max;
        if (velocity > velocity_d) velocity = velocity_d;
        if (ms / 1000.0f < T && velocity > velocity_a) velocity = velocity_a;
        float theta = atan2f(-cur.y, SEARCH_LOOK_AHEAD + velocity / 60) - cur.theta;
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
            float theta = atan2f(-cur.y, SEARCH_LOOK_AHEAD) - cur.theta;
            sc.set_target(velocity, SEARCH_PROP_GAIN * theta);
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

extern SearchRun sr;

