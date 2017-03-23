#pragma once

#include <Arduino.h>
#include <vector>
#include <queue>
#include "TaskBase.h"
#include "config.h"

#include "as5145.h"
#include "motor.h"
#include "mpu6500.h"
#include "reflector.h"
#include "WallDetector.h"
#include "SpeedController.h"

extern AS5145 as;
extern Motor mt;
extern MPU6500 mpu;
extern Reflector ref;
extern WallDetector wd;
extern SpeedController sc;

#define WALL_ATTACH_ENABLED     false
#define WALL_AVOID_ENABLED      true

#define LOOK_AHEAD_UNIT         3
#define TRAJECTORY_PROP_GAIN    120
#define TRAJECTORY_INT_GAIN     0
#define WALL_AVOID_GAIN         0.0001f

class Timer {
  public:
    Timer(): start_us(0), active(false) {}
    void start() {
      if (!active) {
        start_us = micros();
        active = true;
      }
    }
    void reset() {
      start_us = micros();
    }
    float read() {
      return (float)(micros() - start_us) / 1000000;
    }
    uint32_t read_ms() {
      return (micros() - start_us) / 1000;
    }
    uint32_t read_us() {
      return micros() - start_us;
    }
  private:
    uint32_t start_us;
    bool active;
};

class Trajectory {
  public:
    Trajectory() {
      reset();
    }
    virtual ~Trajectory() {
    }
    void reset() {
      last_index = 0;
    }
    Position getNextDir(const Position &cur, float velocity) {
      int index_cur = getNextIndex(cur);
      int look_ahead = LOOK_AHEAD_UNIT * (1.0f + pow(velocity / 600, 2));
      Position dir = (getPosition(index_cur + look_ahead) - cur).rotate(-cur.theta);
      dir.theta = atan2f(dir.y, dir.x);
      return dir;
    }
    float getRemain() const {
      return (getSize() - last_index) * interval;
    }
    Position getEndPosition() {
      return getPosition(getSize());
    }
  protected:
    int last_index;
    const float interval = 2.0f;
    virtual int size() const {
      return 180;
    }
    virtual Position position(int index) const {
      return Position(index * interval, 0, 0);
    }
    int getSize() const {
      return size();
    }
    Position getPosition(const int index) {
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

class Curve90: public Trajectory {
  public:
    Curve90(bool mirror = false) :
      Trajectory(), mirror(mirror) {
    }
    const float velocity = 260.0f;
    const float straight = 15.0f;
  private:
    bool mirror;
    virtual int size() const {
      return 26;
    }
    virtual Position position(int index) const {
      static const float data[26 + 1][3] =
      { { 0.0000000000, 0.0000000000, 0.0000000000 }, { 1.9999999343, 0.0003581351, 0.0007153325 }, { 3.9999907677, 0.0056854550, 0.0056557394 }, { 5.9998473365, 0.0284088519, 0.0187172793 }, { 7.9989048295, 0.0881534668, 0.0431636004 }, { 9.9950574894, 0.2101682305, 0.0813717929 }, { 11.9834416379, 0.4231979224, 0.1346472802 }, { 13.9550171391, 0.7568522412, 0.2031221621 }, { 15.8955471785, 1.2385834025, 0.2857449285 }, { 17.7855147928, 1.8905302360, 0.3803623424 }, { 19.6013550216, 2.7266811121, 0.4838871173 }, { 21.3180252310, 3.7509398130, 0.5925383310 }, { 22.9124075718, 4.9567247722, 0.7024742720 }, { 24.3648619081, 6.3301689163, 0.8124179843 }, { 25.6578492022, 7.8546872908, 0.9223616965 }, { 26.7758327680, 9.5118234214, 1.0320459270 }, { 27.7078174774, 11.2803284795, 1.1386823358 }, { 28.4522831355, 13.1357157330, 1.2382647117 }, { 29.0182522779, 15.0532748985, 1.3273191357 }, { 29.4240311767, 17.0111878431, 1.4031915235 }, { 29.6947572836, 18.9924659652, 1.4642543049 }, { 29.8593376545, 20.9855072002, 1.5100331663 }, { 29.9473142489, 22.9834898549, 1.5412439863 }, { 29.9860126781, 24.9830869308, 1.5597370888 }, { 29.9981501834, 26.9830439783, 1.5683531544 }, { 29.9999745380, 28.9830427265, 1.5707020100 }, { 30.0000000000, 30.0000000059, 1.5707963268 }, };
      Position ret;
      if (index < 0) {
        ret = Position(0 + interval * index, 0, 0);
      } else if (index > size() - 1) {
        Position end(data[size()][0], data[size()][1], data[size()][2]);
        ret = end
              + Position((index - size()) * interval * cos(end.theta),
                         (index - size()) * interval * sin(end.theta), 0);
      } else {
        ret = Position(data[index][0], data[index][1], data[index][2]);
      }
      if (mirror)
        return ret.mirror_x();
      return ret;
    }
};

class C30: public Trajectory {
  public:
    C30(bool mirror = false) :
      Trajectory(), mirror(mirror) {
    }
    const float velocity = 761.5056368240381;
  private:
    bool mirror;
    virtual int size() const {
      return 33;
    }
    virtual Position position(int index) const {
      static const float data[33 + 1][3] =
      { { 0.0000000000, 0.0000000000, 0.0000000000 }, { 1.9999999986, 0.0000502581, 0.0001004536 }, { 3.9999998169, 0.0008011255, 0.0007991263 }, { 5.9999969173, 0.0040304778, 0.0026719061 }, { 7.9999773716, 0.0126275827, 0.0062508522 }, { 9.9998948870, 0.0304850952, 0.0120043334 }, { 11.9996352112, 0.0623531676, 0.0203195445 }, { 13.9989665875, 0.1136605100, 0.0314880521 }, { 15.9974804901, 0.1903081559, 0.0456949109 }, { 17.9945300480, 0.2984425368, 0.0630117555 }, { 19.9891758083, 0.4442153936, 0.0833941265 }, { 21.9801493466, 0.6335391051, 0.1066831300 }, { 23.9658444537, 0.8718471322, 0.1326113691 }, { 25.9443431852, 1.1638702507, 0.1608129257 }, { 27.9134801831, 1.5134397327, 0.1908370198 }, { 29.8709438578, 1.9233282844, 0.2221648374 }, { 31.8144079321, 2.3951380854, 0.2542288968 }, { 33.7416822788, 2.9292426264, 0.2864342331 }, { 35.6508686661, 3.5247854008, 0.3181806074 }, { 37.5405055286, 4.1797343225, 0.3488849138 }, { 39.4096864645, 4.8909866270, 0.3780029461 }, { 41.2581397367, 5.6545156125, 0.4050497129 }, { 43.0862602190, 6.4655484034, 0.4296175421 }, { 44.8950903243, 7.3187632191, 0.4513912987 }, { 46.6862517485, 8.2084953458, 0.4701601493 }, { 48.4618346500, 9.1289427965, 0.4858254319 }, { 50.2242546243, 10.0743649553, 0.4984043388 }, { 51.9760901761, 11.0392697698, 0.5080292754 }, { 53.7199142065, 12.0185867809, 0.5149429169 }, { 55.4581323950, 13.0078241819, 0.5194891500 }, { 57.1928394398, 14.0032081254, 0.5221002350 }, { 58.9257012620, 15.0018018568, 0.5232806708 }, { 60.6578678534, 16.0016012708, 0.5235883613 }, { 61.4711431703, 16.4711431703, 0.5235987756 }, };
      Position ret;
      if (index < 0) {
        ret = Position(0 + interval * index, 0, 0);
      } else if (index > size() - 1) {
        Position end(data[size()][0], data[size()][1], data[size()][2]);
        ret = end
              + Position((index - size()) * interval * cos(end.theta),
                         (index - size()) * interval * sin(end.theta), 0);
      } else {
        ret = Position(data[index][0], data[index][1], data[index][2]);
      }
      if (mirror)
        return ret.mirror_x();
      return ret;
    }
};

class C60: public Trajectory {
  public:
    C60(bool mirror = false) :
      Trajectory(), mirror(mirror) {
    }
    const float velocity = 530.4534632991004f;
    const float straight1 = 30.0f;
    const float straight2 = 17.942286340599491f;
  private:
    bool mirror;
    virtual int size() const {
      return 32;
    }
    virtual Position position(int index) const {
      static const float data[32 + 1][3] =
      { { 0.0000000000, 0.0000000000, 0.0000000000 }, { 1.9999999937, 0.0001051362, 0.0002101373 }, { 3.9999991991, 0.0016757028, 0.0016713942 }, { 5.9999865195, 0.0084288816, 0.0055867697 }, { 7.9999011082, 0.0264006904, 0.0130648724 }, { 9.9995410013, 0.0637124598, 0.0250772813 }, { 11.9984086798, 0.1302532051, 0.0424211768 }, { 13.9954974873, 0.2372846201, 0.0656886795 }, { 15.9890389147, 0.3969756498, 0.0952440761 }, { 17.9762452012, 0.6218750465, 0.1312098082 }, { 19.9530919049, 0.9243344434, 0.1734617630 }, { 21.9141887720, 1.3159019568, 0.2216340466 }, { 23.8527827301, 1.8067164791, 0.2751330578 }, { 25.7609236015, 2.4049433852, 0.3331603181 }, { 27.6298021565, 3.1162999369, 0.3947431838 }, { 29.4502442467, 3.9437196189, 0.4587722552 }, { 31.2133184503, 4.8871964366, 0.5240440468 }, { 32.9109932326, 5.9438325418, 0.5893072754 }, { 34.5367678981, 7.1080879062, 0.6533109872 }, { 36.0862024589, 8.3722039727, 0.7148526719 }, { 37.5572849314, 9.7267502139, 0.7728245139 }, { 38.9505974406, 11.1612285472, 0.8262560007 }, { 40.2692697467, 12.6646685566, 0.8743512514 }, { 41.5187347623, 14.2261563424, 0.9165196271 }, { 42.7063207912, 15.8352585984, 0.9523984490 }, { 43.8407272220, 17.4823264197, 0.9818669468 }, { 44.9314342425, 19.1586852472, 1.0050509044 }, { 45.9880945420, 20.8567340000, 1.0223178219 }, { 47.0199483918, 22.5699853073, 1.0342627806 }, { 48.0352951664, 24.2930791963, 1.0416855551 }, { 49.0410456917, 26.0217957360, 1.0455598525 }, { 50.0423713382, 27.7530803061, 1.0469958614 }, { 51.0288568297, 29.4615242271, 1.0471975512 }, };
      Position ret;
      if (index < 0) {
        ret = Position(0 + interval * index, 0, 0);
      } else if (index > size() - 1) {
        Position end(data[size()][0], data[size()][1], data[size()][2]);
        ret = end
              + Position((index - size()) * interval * cos(end.theta),
                         (index - size()) * interval * sin(end.theta), 0);
      } else {
        ret = Position(data[index][0], data[index][1], data[index][2]);
      }
      if (mirror)
        return ret.mirror_x();
      return ret;
    }
};

class C90: public Trajectory {
  public:
    C90(bool mirror = false) :
      Trajectory(), mirror(mirror) {
    }
    const float velocity = 514.3419901132443f;
  private:
    bool mirror;
    virtual int size() const {
      return 39;
    }
    virtual Position position(int index) const {
      static const float data[39 + 1][3] =
      { { 0.0000000000, 0.0000000000, 0.0000000000 }, { 1.9999999935, 0.0001062679, 0.0002124124 }, { 3.9999991810, 0.0016943744, 0.0016904409 }, { 5.9999861950, 0.0085281212, 0.0056557394 }, { 7.9998985213, 0.0267348438, 0.0132435644 }, { 9.9995277615, 0.0645911425, 0.0254633412 }, { 11.9983575484, 0.1322298235, 0.0431636004 }, { 13.9953353179, 0.2412722497, 0.0670025036 }, { 15.9885949741, 0.4043895846, 0.0974249818 }, { 17.9751624751, 0.6347967984, 0.1346472802 }, { 19.9506881998, 0.9456864600, 0.1786494467 }, { 21.9092563637, 1.3496163393, 0.2291760251 }, { 23.8433208395, 1.8578755425, 0.2857449286 }, { 25.7438072475, 2.4798665685, 0.3476641851 }, { 27.6004031342, 3.2225520918, 0.4140559739 }, { 29.4020331505, 4.0900214815, 0.4838871173 }, { 31.1374879887, 5.0832294844, 0.5560049689 }, { 32.7961494920, 6.1999463467, 0.6291784639 }, { 34.3686106467, 7.4350882329, 0.7024742721 }, { 35.8464001597, 8.7820654850, 0.7757700802 }, { 37.2215824920, 10.2336450092, 0.8490658884 }, { 38.4867730922, 11.7820320109, 0.9223616966 }, { 39.6351873590, 13.4189057067, 0.9956183107 }, { 40.6611869045, 15.1351723936, 1.0681348253 }, { 41.5617284336, 16.9204917719, 1.1386823359 }, { 42.3369256759, 18.7637373574, 1.2060833124 }, { 42.9900617105, 20.6537325671, 1.2692695221 }, { 43.5273801673, 22.5799114522, 1.3273191357 }, { 43.9576922460, 24.5328384473, 1.3794887476 }, { 44.2918495544, 26.5045485770, 1.4252391988 }, { 44.5421380654, 28.4886977649, 1.4642543050 }, { 44.7216442960, 30.4805390650, 1.4964518395 }, { 44.8436343728, 32.4767607366, 1.5219863895 }, { 44.9209739522, 34.4752338791, 1.5412439863 }, { 44.9656055142, 36.4747204317, 1.5548286962 }, { 44.9880914990, 38.4745876926, 1.5635416378 }, { 44.9972276576, 40.4745648950, 1.5683531544 }, { 44.9997300219, 42.4745629908, 1.5703691085 }, { 44.9999994932, 44.4745629578, 1.5707924688 }, { 45.0000000000, 45.0000000000, 1.5707963268 }, };
      Position ret;
      if (index < 0) {
        ret = Position(0 + interval * index, 0, 0);
      } else if (index > size() - 1) {
        Position end(data[size()][0], data[size()][1], data[size()][2]);
        ret = end
              + Position((index - size()) * interval * cos(end.theta),
                         (index - size()) * interval * sin(end.theta), 0);
      } else {
        ret = Position(data[index][0], data[index][1], data[index][2]);
      }
      if (mirror)
        return ret.mirror_x();
      return ret;
    }
};

class C120: public Trajectory {
  public:
    C120(bool mirror = false) :
      Trajectory(), mirror(mirror) {
    }
    const float velocity = 704.6972922783409f;
  private:
    bool mirror;
    virtual int size() const {
      return 63;
    }
    virtual Position position(int index) const {
      static const float data[63 + 1][3] =
      { { 0.0000000000, 0.0000000000, 0.0000000000 }, { 1.9999999990, 0.0000413415, 0.0000826574 }, { 3.9999998756, 0.0006602372, 0.0006594205 }, { 5.9999978890, 0.0033321302, 0.0022152395 }, { 7.9999843446, 0.0104856826, 0.0052169232 }, { 9.9999263123, 0.0254576830, 0.0101044889 }, { 11.9997401200, 0.0524308557, 0.0172830094 }, { 13.9992496628, 0.0963552816, 0.0271151073 }, { 15.9981301705, 0.1628541051, 0.0399142351 }, { 17.9958388359, 0.2581140974, 0.0559388679 }, { 19.9915354857, 0.3887615574, 0.0753877133 }, { 21.9839981467, 0.5617240745, 0.0983960299 }, { 23.9715397780, 0.7840789367, 0.1250331205 }, { 25.9519334705, 1.0628895446, 0.1553010484 }, { 27.9223539325, 1.4050321029, 0.1891345993 }, { 29.8793429753, 1.8170160822, 0.2264024890 }, { 31.8188059100, 2.3048033702, 0.2669097965 }, { 33.7360442317, 2.8736324754, 0.3104015761 }, { 35.6258277554, 3.5278553969, 0.3565675808 }, { 37.4825065864, 4.2707955606, 0.4050480113 }, { 39.3001601791, 5.1046353352, 0.4554401831 }, { 41.0727775444, 6.0303408998, 0.5073059881 }, { 42.7944597407, 7.0476305960, 0.5601800119 }, { 44.4596334724, 8.1549904225, 0.6135781589 }, { 46.0632435928, 9.3497626605, 0.6670750275 }, { 47.6006734480, 10.6285728728, 0.7205719151 }, { 49.0675240904, 11.9877620838, 0.7740688028 }, { 50.4595985164, 13.4234413346, 0.8275656904 }, { 51.7729136746, 14.9315028102, 0.8810625780 }, { 53.0037118623, 16.5076315929, 0.9345594656 }, { 54.1484714773, 18.1473180089, 0.9880563532 }, { 55.2039170938, 19.8458705306, 1.0415532408 }, { 56.1670288346, 21.5984292011, 1.0950501284 }, { 57.0350510109, 23.3999795390, 1.1485470160 }, { 57.8055000079, 25.2453668863, 1.2020439036 }, { 58.4761713903, 27.1293111572, 1.2555407912 }, { 59.0451462099, 29.0464219451, 1.3090376788 }, { 59.5107964965, 30.9912139467, 1.3625345664 }, { 59.8717899152, 32.9581226559, 1.4160314541 }, { 60.1270935794, 34.9415202856, 1.4695283417 }, { 60.2759985693, 36.9357306423, 1.5229744273 }, { 60.3184977766, 38.9350446209, 1.5759998096 }, { 60.2557990782, 40.9338350662, 1.6281178663 }, { 60.0903549063, 42.9267650274, 1.6788585093 }, { 59.8257772802, 44.9089862103, 1.7277771733 }, { 59.4667159193, 46.8763062766, 1.7744630534 }, { 59.0187078757, 48.8253164984, 1.8185467174 }, { 58.4880083048, 50.7534744539, 1.8597069510 }, { 57.8814121708, 52.6591398067, 1.8976767098 }, { 57.2060758958, 54.5415644176, 1.9322480676 }, { 56.4693464363, 56.4008408097, 1.9632760691 }, { 55.6786032513, 58.2378151680, 1.9906814159 }, { 54.8411164445, 60.0539724982, 2.0144519358 }, { 53.9639222826, 61.8513023154, 2.0346428086 }, { 53.0537155635, 63.6321533402, 2.0513755433 }, { 52.1167570799, 65.3990852544, 2.0648357266 }, { 51.1587937680, 67.1547247372, 2.0752695845 }, { 50.1849890402, 68.9016318705, 2.0829794194 }, { 49.1998611959, 70.6421816765, 2.0883180086 }, { 48.2072285464, 72.3784640988, 2.0916820677 }, { 47.2101608400, 74.1122042411, 2.0935048994 }, { 46.2109375500, 75.8447031971, 2.0942483647 }, { 45.2110144628, 77.5767984059, 2.0943943251 }, { 45.0000000000, 77.9422863406, 2.0943951024 }, };
      Position ret;
      if (index < 0) {
        ret = Position(0 + interval * index, 0, 0);
      } else if (index > size() - 1) {
        Position end(data[size()][0], data[size()][1], data[size()][2]);
        ret = end
              + Position((index - size()) * interval * cos(end.theta),
                         (index - size()) * interval * sin(end.theta), 0);
      } else {
        ret = Position(data[index][0], data[index][1], data[index][2]);
      }
      if (mirror)
        return ret.mirror_x();
      return ret;
    }
};

class C150: public Trajectory {
  public:
    C150(bool mirror = false) :
      Trajectory(), mirror(mirror) {
    }
    const float velocity = 740.2288193534528f;
    const float straight1 = 25.0f;
    const float straight2 = 4.115427290353293f;
  private:
    bool mirror;
    virtual int size() const {
      return 76;
    }
    virtual Position position(int index) const {
      static const float data[76 + 1][3] =
      { { 0.0000000000, 0.0000000000, 0.0000000000 }, { 1.9999999981, 0.0000356715, 0.0000713229 }, { 3.9999999061, 0.0005696680, 0.0005691455 }, { 5.9999983753, 0.0028765993, 0.0019128041 }, { 7.9999883910, 0.0090550461, 0.0045074283 }, { 9.9999446869, 0.0219970404, 0.0087371369 }, { 11.9998058281, 0.0453317399, 0.0149585860 }, { 13.9994375794, 0.0833721760, 0.0234949788 }, { 15.9985959429, 0.1410350699, 0.0346306369 }, { 17.9968682969, 0.2237558547, 0.0486062253 }, { 19.9936125615, 0.3373966350, 0.0656147099 }, { 21.9878913689, 0.4881243647, 0.0857981165 }, { 23.9783956088, 0.6823092704, 0.1092451455 }, { 25.9633876441, 0.9263770741, 0.1359896820 }, { 27.9406369575, 1.2266835573, 0.1660102266 }, { 29.9073840710, 1.5893665240, 0.1992302568 }, { 31.8603179624, 2.0201892627, 0.2355195120 }, { 33.7955839682, 2.5244094806, 0.2746961812 }, { 35.7088133836, 3.1066160691, 0.3165299570 }, { 37.5952001096, 3.7706165162, 0.3607459038 }, { 39.4495840574, 4.5193121906, 0.4070290752 }, { 41.2665883119, 5.3546123260, 0.4550298042 }, { 43.0407645118, 6.2773752942, 0.5043695759 }, { 44.7667531758, 7.2873820964, 0.5546473851 }, { 46.4394762801, 8.3833474147, 0.6054464714 }, { 48.0542836542, 9.5629798285, 0.6563746676 }, { 49.6069458429, 10.8232877930, 0.7073036653 }, { 51.0934360732, 12.1610027201, 0.7582326630 }, { 52.5099002107, 13.5726562187, 0.8091616607 }, { 53.8526645958, 15.0545871272, 0.8600906584 }, { 55.1182472438, 16.6029525655, 0.9110196561 }, { 56.3033665343, 18.2137376761, 0.9619486538 }, { 57.4049487494, 19.8827647221, 1.0128776515 }, { 58.4201376477, 21.6057061591, 1.0638066492 }, { 59.3463005452, 23.3780939348, 1.1147356470 }, { 60.1810354822, 25.1953313882, 1.1656646447 }, { 60.9221781863, 27.0527068505, 1.2165936424 }, { 61.5678065027, 28.9454032746, 1.2675226401 }, { 62.1162461622, 30.8685123927, 1.3184516378 }, { 62.5660751244, 32.8170478203, 1.3693806355 }, { 62.9161267130, 34.7859558328, 1.4203096332 }, { 63.1654932693, 36.7701311770, 1.4712386309 }, { 63.3135281248, 38.7644285309, 1.5221676286 }, { 63.3598473688, 40.7636756207, 1.5730966263 }, { 63.3043309060, 42.7626888499, 1.6240256240 }, { 63.1471226874, 44.7562839628, 1.6749546217 }, { 62.8886304327, 46.7392908487, 1.7258836194 }, { 62.5295243611, 48.7065679478, 1.7768126171 }, { 62.0707358352, 50.6530129699, 1.8277416148 }, { 61.5134545174, 52.5735787085, 1.8786706125 }, { 60.8591255236, 54.4632848562, 1.9295996103 }, { 60.1094457181, 56.3172308190, 1.9805286080 }, { 59.2663622931, 58.1306106068, 2.0314425569 }, { 58.3322537436, 59.8988269028, 2.0820974506 }, { 57.3103337796, 61.6177919913, 2.1320749286 }, { 56.2046595780, 63.2841334509, 2.1809632750 }, { 55.0200132318, 64.8953001289, 2.2283690667 }, { 53.7617281813, 66.4496583806, 2.2739237811 }, { 52.4355522301, 67.9465319030, 2.3172899863 }, { 51.0474750205, 69.3862137237, 2.3581670082 }, { 49.6035821713, 70.7699366196, 2.3962959804 }, { 48.1099262620, 72.0998162529, 2.4314641926 }, { 46.5723999405, 73.3787491241, 2.4635086627 }, { 44.9966561983, 74.6103157964, 2.4923188746 }, { 43.3880272983, 75.7986319918, 2.5178386342 }, { 41.7514814320, 76.9482195488, 2.5400670134 }, { 40.0915904180, 78.0638557220, 2.5590583656 }, { 38.4125146465, 79.1504180019, 2.5749214126 }, { 36.7179991521, 80.2127573497, 2.5878174189 }, { 35.0113762800, 81.2555432937, 2.5979574829 }, { 33.2955704953, 82.2831575678, 2.6055989913 }, { 31.5731038454, 83.2995724842, 2.6110412961 }, { 29.8460982684, 84.3082581118, 2.6146206869 }, { 28.1162747145, 85.3121056317, 2.6167047425 }, { 26.3849478311, 86.3133581347, 2.6176861569 }, { 24.6530196485, 87.3135704867, 2.6179761442 }, { 23.5640645809, 87.9422863548, 2.6179938780 }, };
      Position ret;
      if (index < 0) {
        ret = Position(0 + interval * index, 0, 0);
      } else if (index > size() - 1) {
        Position end(data[size()][0], data[size()][1], data[size()][2]);
        ret = end
              + Position((index - size()) * interval * cos(end.theta),
                         (index - size()) * interval * sin(end.theta), 0);
      } else {
        ret = Position(data[index][0], data[index][1], data[index][2]);
      }
      if (mirror)
        return ret.mirror_x();
      return ret;
    }
};

class C180: public Trajectory {
  public:
    C180(bool mirror = false) :
      Trajectory(), mirror(mirror) {
    }
    const float velocity = 818.9712226221780f;
    //  const float velocity = 600.0f;
    const float straight = 20.0f;
  private:
    bool mirror;
    virtual int size() const {
      return 96;
    }
    virtual Position position(int index) const {
      static const float data[96 + 1][3] =
      { { 0.0000000000, 0.0000000000, 0.0000000000 }, { 1.9999999996, 0.0000263425, 0.0000526730 }, { 3.9999999494, 0.0004209018, 0.0004205163 }, { 5.9999991403, 0.0021259420, 0.0014143745 }, { 7.9999936075, 0.0066975236, 0.0033365004 }, { 9.9999698099, 0.0162841234, 0.0064764061 }, { 11.9998930869, 0.0335970499, 0.0111068903 }, { 13.9996898093, 0.0618729410, 0.0174802934 }, { 15.9992226526, 0.1048286326, 0.0258250345 }, { 17.9982590072, 0.1666086669, 0.0363424766 }, { 19.9964331596, 0.2517256656, 0.0492041624 }, { 21.9932035091, 0.3649937609, 0.0645494605 }, { 23.9878066746, 0.5114552814, 0.0824836519 }, { 25.9792108754, 0.6963009564, 0.1030764858 }, { 27.9660713915, 0.9247840528, 0.1263612216 }, { 29.9466911945, 1.2021291170, 0.1523341729 }, { 31.9189899589, 1.5334363503, 0.1809547578 }, { 33.8804845816, 1.9235830950, 0.2121460562 }, { 35.8282840496, 2.3771244094, 0.2457958652 }, { 37.7591009805, 2.8981952316, 0.2817582392 }, { 39.6692814385, 3.4904170907, 0.3198554929 }, { 41.5548537162, 4.1568126847, 0.3598806402 }, { 43.4115957266, 4.8997318140, 0.4016002350 }, { 45.2351195243, 5.7207921063, 0.4447575749 }, { 47.0209703598, 6.6208376451, 0.4890762248 }, { 48.7647366504, 7.5999180172, 0.5342638109 }, { 50.4621664146, 8.6572894416, 0.5800160331 }, { 52.1092851222, 9.7914386235, 0.6260212589 }, { 53.7024764159, 11.0001715253, 0.6720535380 }, { 55.2383589628, 12.2809363471, 0.7180858170 }, { 56.7136788472, 13.6310196649, 0.7641180960 }, { 58.1253104615, 15.0475611968, 0.8101503751 }, { 59.4702631279, 16.5275598629, 0.8561826541 }, { 60.7456874342, 18.0678801430, 0.9022149332 }, { 61.9488812709, 19.6652587198, 0.9482472122 }, { 63.0772955556, 21.3163113926, 0.9942794912 }, { 64.1285396337, 23.0175402469, 1.0403117703 }, { 65.1003863431, 24.7653410650, 1.0863440493 }, { 65.9907767327, 26.5560109621, 1.1323763283 }, { 66.7978244245, 28.3857562311, 1.1784086074 }, { 67.5198196100, 30.2507003800, 1.2244408864 }, { 68.1552326727, 32.1468923444, 1.2704731655 }, { 68.7027174286, 34.0703148587, 1.3165054445 }, { 69.1611139784, 36.0168929664, 1.3625377235 }, { 69.5294511648, 37.9825026538, 1.4085700026 }, { 69.8069486299, 39.9629795871, 1.4546022816 }, { 69.9930184684, 41.9541279346, 1.5006345606 }, { 70.0872664735, 43.9517292561, 1.5466668397 }, { 70.0894929716, 45.9515514405, 1.5926991187 }, { 69.9996932458, 47.9493576712, 1.6387313977 }, { 69.8180575454, 49.9409154029, 1.6847636768 }, { 69.5449706831, 51.9220053281, 1.7307959558 }, { 69.1810112199, 53.8884303166, 1.7768282349 }, { 68.7269502390, 55.8360243072, 1.8228605139 }, { 68.1837497125, 57.7606611338, 1.8688927929 }, { 67.5525604632, 59.6582632675, 1.9149250720 }, { 66.8347197263, 61.5248104549, 1.9609573510 }, { 66.0317483169, 63.3563482356, 2.0069896300 }, { 65.1453474074, 65.1489963198, 2.0530219091 }, { 64.1773949238, 66.8989568096, 2.0990541881 }, { 63.1299415668, 68.6025222445, 2.1450864672 }, { 62.0052064675, 70.2560834569, 2.1911187462 }, { 60.8055724860, 71.8561372174, 2.2371510252 }, { 59.5335811625, 73.3992936577, 2.2831833043 }, { 58.1919273336, 74.8822834519, 2.3292155833 }, { 56.7834534224, 76.3019647429, 2.3752478623 }, { 55.3111434170, 77.6553297987, 2.4212801414 }, { 53.7781165483, 78.9395113848, 2.4673124204 }, { 52.1876206818, 80.1517888385, 2.5133446995 }, { 50.5430301282, 81.2896008531, 2.5593545365 }, { 48.8479452667, 82.3507270721, 2.6051262041 }, { 47.1063242948, 83.3336180147, 2.6503481858 }, { 45.3224084749, 84.2374919446, 2.6947157097 }, { 43.5006043813, 85.0623598636, 2.7379357310 }, { 41.6453672770, 85.8090277330, 2.7797309544 }, { 39.7610907224, 86.4790765589, 2.8198436396 }, { 37.8520069016, 87.0748219564, 2.8580391389 }, { 35.9221013299, 87.5992556729, 2.8941091178 }, { 33.9750445904, 88.0559721590, 2.9278744143 }, { 32.0141426340, 88.4490836153, 2.9591874977 }, { 30.0423060549, 88.7831270055, 2.9879344935 }, { 28.0620376956, 89.0629663682, 3.0140367467 }, { 26.0754370215, 89.2936934081, 3.0374519008 }, { 24.0842189694, 89.4805288873, 3.0581744793 }, { 22.0897444499, 89.6287268238, 3.0762359610 }, { 20.0930593854, 89.7434829945, 3.0917043479 }, { 18.0949390723, 89.8298487941, 3.1046832319 }, { 16.0959347671, 89.8926511340, 3.1153103728 }, { 14.0964196733, 89.9364188077, 3.1237558079 }, { 12.0966319230, 89.9653155915, 3.1302195180 }, { 10.0967126691, 89.9830802769, 3.1349286828 }, { 8.0967379965, 89.9929738290, 3.1381345630 }, { 6.0967439860, 89.9977338934, 3.1401090515 }, { 4.0967448875, 89.9995369183, 3.1411409420 }, { 2.0967449467, 89.9999681800, 3.1415319652 }, { 0.0967449473, 89.9999999999, 3.1415926476 }, { 0.0000000000, 90.0000000000, 3.1415926536 }, };
      Position ret;
      if (index < 0) {
        ret = Position(0 + interval * index, 0, 0);
      } else if (index > size() - 1) {
        Position end(data[size()][0], data[size()][1], data[size()][2]);
        ret = end
              + Position((index - size()) * interval * cos(end.theta),
                         (index - size()) * interval * sin(end.theta), 0);
      } else {
        ret = Position(data[index][0], data[index][1], data[index][2]);
      }
      if (mirror)
        return ret.mirror_x();
      return ret;
    }
};

#define MOVE_ACTION_TASK_PRIORITY   3
#define MOVE_ACTION_STACK_SIZE      4096

#define MOVE_ACTION_PERIOD          1000

class MoveAction: TaskBase {
  public:
    MoveAction() : TaskBase("Move Action Task", MOVE_ACTION_TASK_PRIORITY, MOVE_ACTION_STACK_SIZE) {
      set_params(600);
    }
    virtual ~MoveAction() {}
    enum ACTION {
      START_STEP, START_INIT, GO_STRAIGHT, GO_HALF, TURN_LEFT_90, TURN_RIGHT_90, RETURN, STOP,
    };
    enum FAST_ACTION
      : char {
      FAST_GO_STRAIGHT = 's',
      FAST_GO_HALF = 'x',
      FAST_TURN_LEFT_30 = 'w',
      FAST_TURN_RIGHT_30 = 'W',
      FAST_TURN_LEFT_60 = 'z',
      FAST_TURN_RIGHT_60 = 'c',
      FAST_TURN_LEFT_60R = 'Z',
      FAST_TURN_RIGHT_60R = 'C',
      FAST_TURN_LEFT_120 = 'q',
      FAST_TURN_RIGHT_120 = 'e',
      FAST_TURN_LEFT_90 = 'L',
      FAST_TURN_RIGHT_90 = 'R',
      FAST_TURN_LEFT_150 = 'a',
      FAST_TURN_RIGHT_150 = 'd',
      FAST_TURN_LEFT_150R = 'A',
      FAST_TURN_RIGHT_150R = 'D',
      FAST_TURN_LEFT_180 = 'Q',
      FAST_TURN_RIGHT_180 = 'E',
    };
    struct Operation {
      enum ACTION action;
      int num;
    };
    const char* action_string(enum ACTION action) {
      static const char name[][32] =
      { "start_step", "start_init", "go_straight", "go_half", "turn_left_90", "turn_right_90", "return", "stop", };
      return name[action];
    }
    void enable(const float speed = 200) {
      printf("Move Action Enabled\n");
      if (path.length() > 0) {
        this->fast_speed = speed;
      } else {
        this->search_speed = speed;
      }
      create_task();
    }
    void disable() {
      delete_task();
      sc.disable();
      ref.disable();
      while (q.size()) {
        q.pop();
      }
      path = "";
    }
    void set_action(FAST_ACTION action, const int num = 1) {
      for (int i = 0; i < num; i++)
        path += (char)action;
    }
    void set_action(String actions) {
      path = actions;
    }
    void set_action(enum ACTION action, int num = 1) {
      struct Operation operation;
      operation.action = action;
      operation.num = num;
      q.push(operation);
    }
    void set_params(float fast_speed) {
      this->fast_speed = fast_speed;
    }
    void set_params_relative(float add) {
      this->fast_speed += add;
    }
    int actions() const {
      return q.size() + path.length();
    }
    void printPosition(const char* name) {
      printf("%s\t", name);
      //    printf("Ori:(%06.1f, %06.1f, %06.3f)\t", origin.x, origin.y, origin.theta);
      //    printf("Abs:(%06.1f, %06.1f, %06.3f)\t", sc.getPosition().x, sc.getPosition().y,
      //        sc.getPosition().theta);
      printf("Rel:(%06.1f, %06.1f, %06.3f)\t", getRelativePosition().x, getRelativePosition().y,
             getRelativePosition().theta);
      printf("\n");
    }
    Position getRelativePosition() {
      return (sc.getPosition() - origin).rotate(-origin.theta);
    }
    void updateOrigin(Position passed) {
      origin += passed.rotate(origin.theta);
    }
    void setPosition(Position pos = Position(SEGMENT_WIDTH / 2, WALL_THICKNESS / 2 + MACHINE_TAIL_LENGTH, M_PI / 2)) {
      origin = pos;
      sc.getPosition() = pos;
    }
    void fixPosition(Position pos) {
      sc.getPosition() -= pos;
    }
  private:
    portTickType xLastWakeTime;
    float fast_speed;
    float search_speed;
    Position origin;
    std::queue<struct Operation> q;
    String path;
    Timer timer;

    void wall_avoid() {
#if WALL_AVOID_ENABLED
      const float gain = WALL_AVOID_GAIN;
      if (wd.wall().side[0]) {
        fixPosition(Position(0, wd.wall_difference().side[0] * gain * sc.actual.trans, 0).rotate(origin.theta));
      }
      if (wd.wall().side[1]) {
        fixPosition(Position(0, -wd.wall_difference().side[1] * gain * sc.actual.trans, 0).rotate(origin.theta));
      }
#endif
    }
    void wall_attach() {
#if WALL_ATTACH_ENABLED
      if (wd->wall().flont[0] && wd->wall().flont[1]) {
        while (1) {
          float trans = wd->wall_difference().flont[0] + wd->wall_difference().flont[1];
          float rot = wd->wall_difference().flont[1] - wd->wall_difference().flont[0];
          sc.set_target(trans * 100, rot * 10);
          if (fabs(trans) < 0.1f && fabs(rot) < 0.1f)
            break;
          delay(1);
        }
        sc.set_target(0, 0);
        printPosition("1");
        fixPosition(Position(getRelativePosition().x, 0, 0).rotate(origin.theta));
        printPosition("2");
        bz->play(Buzzer::SELECT);
        delay(1000);
        while (1) {
          Position cur = getRelativePosition();
          if (fabs(10 - cur.x) < 0.1)
            break;
          Thread::signal_wait(0x01);
          sc.set_target((10 - cur.x) * 100, -cur.y * 1);
        }
        sc.set_target(0, 0);
        printPosition("3");
        fixPosition(Position(getRelativePosition().x, 0, 0).rotate(origin.theta));
        printPosition("4");
        bz->play(Buzzer::SELECT);
        Thread::wait(1000);
      }
#endif
    }
    void turn(const float angle) {
      const float speed = 3 * M_PI;
      const float accel = 48 * M_PI;
      timer.reset();
      timer.start();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        if (fabs(sc.actual.rot) > speed)
          break;
        if (angle > 0) {
          sc.set_target(0, timer.read() * accel);
        } else {
          sc.set_target(0, -timer.read() * accel);
        }
      }
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        if (fabs(sc.actual.rot) < 0.2)
          break;
        float extra = angle - getRelativePosition().theta;
        float target_speed = sqrt(2 * accel * fabs(extra));
        target_speed = (target_speed > speed) ? speed : target_speed;
        if (extra > 0) {
          sc.set_target(0, target_speed);
        } else {
          sc.set_target(0, -target_speed);
        }
      }
      updateOrigin(Position(0, 0, angle));
    }
    void straight_x(const float distance, const float v_max, const float v_end, bool avoid = true) {
      const float accel = 9000;
      const float decel = 6000;
      timer.reset();
      timer.start();
      float v_start = sc.actual.trans;
      float T = 1.5f * (v_max - v_start) / accel;
      while (1) {
        Position cur = getRelativePosition();
        if (cur.x > distance - 5.0f)
          break;
        if (v_end < 1.0f && cur.x > distance - 10.0f && sc.actual.trans < 1.0f)
          break;
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        float extra = distance - cur.x;
        float velocity_a = v_start
                           + (v_max - v_start) * 6.0f
                           * (-1.0f / 3 * pow(timer.read() / T, 3) + 1.0f / 2 * pow(timer.read() / T, 2));
        float velocity_d = sqrt(2 * decel * fabs(extra) + v_end * v_end);
        float velocity = v_max;
        if (velocity > velocity_d)
          velocity = velocity_d;
        if (timer.read() < T && velocity > velocity_a)
          velocity = velocity_a;
#define LOOK_AHEAD_UNIT_ST    10
#define TRAJECTORY_PROP_GAIN_ST 40
        float theta = atan2f(-cur.y, 10 + LOOK_AHEAD_UNIT_ST * pow(velocity / 900, 2)) - cur.theta;
        sc.set_target(velocity, TRAJECTORY_PROP_GAIN_ST * theta);
        if (avoid)
          wall_avoid();
      }
      sc.set_target(v_end, 0);
      //    printPosition("Straight");
      updateOrigin(Position(distance, 0, 0));
    }
    template<class C>
    void trace(C tr, const float velocity) {
      int cnt = 0;
      float integral = 0;
      while (1) {
        if (tr.getRemain() < 5.0f)
          break;
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        Position dir = tr.getNextDir(getRelativePosition(), velocity);
        integral += dir.theta * TRAJECTORY_INT_GAIN * MOVE_ACTION_PERIOD / 1000000;
        sc.set_target(velocity, (dir.theta + integral) * TRAJECTORY_PROP_GAIN);
        if (cnt++ % 20 == 0) {
          //        printf("%.1f\t%.3f\n", dir.x, dir.theta);
        }
      }
      sc.set_target(velocity, 0);
      updateOrigin(tr.getEndPosition());
    }
    virtual void task() {
      ref.enable();
      sc.enable();
      if (path.length() > 0) {
        fastRun();
      } else {
        searchRun();
      }
      //      disable();
      while (1) {
        delay(1);
      }
    }
    void searchRun() {
      const float velocity = 240;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        while (q.empty()) {
          Curve90 tr;
          vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
          Position cur = getRelativePosition();
          const float decel = 6000;
          float extra = tr.straight - cur.x;
          float v = sqrt(2 * decel * fabs(extra));
          if (v > velocity)
            v = velocity;
          if (extra < 0)
            v = -v;
          float theta = atan2f(-cur.y, 10 + LOOK_AHEAD_UNIT_ST * pow(v / 1200, 2)) - cur.theta;
          sc.set_target(v, TRAJECTORY_PROP_GAIN_ST * theta);
          //        wall_avoid();
        }
        struct Operation operation = q.front();
        enum ACTION action = operation.action;
        int num = operation.num;
        printf("%s: %d\n", action_string(action), num);
        printPosition("Start");
        switch (action) {
          case START_STEP:
            setPosition();
            straight_x(SEGMENT_WIDTH - MACHINE_TAIL_LENGTH - WALL_THICKNESS / 2, velocity, velocity);
            break;
          case START_INIT:
            straight_x(SEGMENT_WIDTH / 2, velocity, 0);
            wall_attach();
            turn(M_PI / 2);
            wall_attach();
            turn(M_PI / 2);
            for (int i = 0; i < 100; i++) {
              sc.set_target(-i, 0);
              delay(1);
            }
            delay(400);
            sc.disable();
            mt.drive(-60, -60);
            delay(400);
            mt.drive(0, 0);
            while (q.size()) {
              q.pop();
            }
            return;
          case GO_STRAIGHT:
            straight_x(SEGMENT_WIDTH * num, 360, velocity);
            break;
          case GO_HALF:
            straight_x(SEGMENT_WIDTH / 2 * num, velocity, velocity);
            break;
          case TURN_LEFT_90:
            for (int i = 0; i < num; i++) {
              Curve90 tr(false);
              straight_x(tr.straight, velocity, tr.velocity);
              trace(tr, tr.velocity);
              straight_x(tr.straight, tr.velocity, velocity);
            }
            break;
          case TURN_RIGHT_90:
            for (int i = 0; i < num; i++) {
              Curve90 tr(true);
              straight_x(tr.straight, velocity, tr.velocity);
              trace(tr, tr.velocity);
              straight_x(tr.straight, tr.velocity, velocity);
            }
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
            straight_x(SEGMENT_WIDTH / 2, velocity, 0);
            wall_attach();
            break;
        }
        q.pop();
        printPosition("End");
      }
    }
    void fastRun() {
      if (path[0] != 'x' && path[0] != 'c' && path[0] != 'z') {
        path = "x" + path + "x";
      }

      printf("Path: %s\n", path.c_str());
      path.replace("s", "xx");
      path.replace("L", "ll");
      path.replace("R", "rr");

      path.replace("rllllr", "rlqlr");
      path.replace("lrrrrl", "lrerl");

      path.replace("xllr", "zlr");
      path.replace("xrrl", "crl");
      path.replace("lrrx", "lrC");
      path.replace("rllx", "rlZ");

      path.replace("xllllr", "alr");
      path.replace("xrrrrl", "drl");
      path.replace("rllllx", "rlA");
      path.replace("lrrrrx", "lrD");

      path.replace("xllllx", "Q");
      path.replace("xrrrrx", "E");

      path.replace("rllr", "rlwlr");
      path.replace("lrrl", "lrWrl");

      path.replace("rl", "");
      path.replace("lr", "");
      path.replace("rr", "R");
      path.replace("ll", "L");
      printf("Path: %s\n", path.c_str());

      const float v_max = 3000;
      const float curve_gain = 1.0f;
      setPosition();
      printPosition("S");
      int path_index = 0;
      float straight = SEGMENT_WIDTH / 2 - MACHINE_TAIL_LENGTH - WALL_THICKNESS / 2;
      while (1) {
        if (path_index > (int) path.length() - 1)
          break;
        switch (path[path_index]) {
          case FAST_TURN_LEFT_60: {
              C60 tr(false);
              straight += tr.straight1;
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
              straight += tr.straight2;
            }
            break;
          case FAST_TURN_RIGHT_60: {
              C60 tr(true);
              straight += tr.straight1;
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
              straight += tr.straight2;
            }
            break;
          case FAST_TURN_LEFT_60R: {
              C60 tr(false);
              straight += tr.straight2;
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain, false);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
              straight += tr.straight1;
            }
            break;
          case FAST_TURN_RIGHT_60R: {
              C60 tr(true);
              straight += tr.straight2;
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain, false);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
              straight += tr.straight1;
            }
            break;
          case FAST_TURN_LEFT_30: {
              C30 tr(false);
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain, false);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
            }
            break;
          case FAST_TURN_RIGHT_30: {
              C30 tr(true);
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain, false);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
            }
            break;
          case FAST_TURN_LEFT_120: {
              C120 tr(false);
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain, false);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
            }
            break;
          case FAST_TURN_RIGHT_120: {
              C120 tr(true);
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain, false);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
            }
            break;
          case FAST_TURN_LEFT_90: {
              C90 tr(false);
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
            }
            break;
          case FAST_TURN_RIGHT_90: {
              C90 tr(true);
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
            }
            break;
          case FAST_TURN_LEFT_150: {
              C150 tr(false);
              straight += tr.straight1;
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
              straight += tr.straight2;
            }
            break;
          case FAST_TURN_RIGHT_150: {
              C150 tr(true);
              straight += tr.straight1;
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
              straight += tr.straight2;
            }
            break;
          case FAST_TURN_LEFT_150R: {
              C150 tr(false);
              straight += tr.straight2;
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain, false);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
              straight += tr.straight1;
            }
            break;
          case FAST_TURN_RIGHT_150R: {
              C150 tr(true);
              straight += tr.straight2;
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain, false);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
              straight += tr.straight1;
            }
            break;
          case FAST_TURN_LEFT_180: {
              C180 tr(false);
              straight += tr.straight;
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
              straight += tr.straight;
            }
            break;
          case FAST_TURN_RIGHT_180: {
              C180 tr(true);
              straight += tr.straight;
              if (straight > 1.0f) {
                straight_x(straight, v_max, tr.velocity * curve_gain);
                straight = 0;
              }
              trace(tr, tr.velocity * curve_gain);
              straight += tr.straight;
            }
            break;
          case FAST_GO_STRAIGHT:
            straight += SEGMENT_WIDTH;
            break;
          case FAST_GO_HALF:
            straight += SEGMENT_WIDTH / 2;
            break;
        }
        path_index++;
      }

      printPosition("E");
      if (straight > 1.0f) {
        straight_x(straight, v_max, 0);
        straight = 0;
      }
      printPosition("E");
      wall_attach();
      sc.set_target(0, 0);
      delay(100);
      sc.disable();
      ref.disable();
      path = "";
      printPosition("E");
    }
};

