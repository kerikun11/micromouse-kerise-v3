/**
* @file RobotBase.h
* @brief ロボットのベース
* @author KERI (Github: kerikun11)
* @url https://kerikeri.top/
* @date 2017.10.30
*/
#pragma once

#include "Agent.h"

namespace MazeLib {
  class RobotBase : public Agent {
  public:
    RobotBase(const Vectors& goals)
    : Agent(goals) { }
    enum Action : char {
      START_STEP,
      START_INIT,
      STOP_HALF,
      TURN_LEFT_90,
      TURN_RIGHT_90,
      ROTATE_LEFT_90,
      ROTATE_RIGHT_90,
      ROTATE_180,
      STRAIGHT_FULL,
      STRAIGHT_HALF,
    };
    bool searchRun()
    {
      if(isComplete()) return true;
      queueAction(START_STEP);
      updateCurVecDir(Vector(0, 1), Dir::North);
      calibration();
      startDequeue();
      auto res = generalSearchRun();
      if(!res) {
        stopDequeue();
        return false;
      }
      queueAction(START_INIT);
      updateCurVecDir(Vector(0, 0), Dir::North);
      calcNextDirs(); //< 時間がかかる処理！
      waitForEndAction();
      stopDequeue();
      backupMazeToFlash();
      return true;
    }
    bool positionIdentifyRun(const Dir start_d)
    {
      positionIdentify(start_d+2);
      queueAction(ROTATE_180);
      queueAction(STRAIGHT_HALF);
      calibration();
      startDequeue();
      auto res = generalSearchRun();
      if(!res) {
        stopDequeue();
        return false;
      }
      queueAction(START_INIT);
      updateCurVecDir(Vector(0, 0), Dir::North);
      calcNextDirs(); //< 時間がかかる処理！
      waitForEndAction();
      stopDequeue();
      backupMazeToFlash();
      return true;
    }
    bool endFastRunBackingToStartRun()
    {
      Vector v = Vector(0, 0);
      for(auto d: getShortestDirs()) v = v.next(d);
      updateCurVecDir(v, getShortestDirs().back());
      updateCurVecDir(getCurVec().next(getCurDir()+Dir::Back), getCurDir()+Dir::Back);
      queueAction(ROTATE_180);
      queueAction(STRAIGHT_HALF);
      calibration();
      startDequeue();
      auto res = generalSearchRun();
      if(!res) {
        stopDequeue();
        return false;
      }
      queueAction(START_INIT);
      updateCurVecDir(Vector(0, 0), Dir::North);
      calcNextDirs(); //< 時間がかかる処理！
      waitForEndAction();
      stopDequeue();
      backupMazeToFlash();
      return true;
    }
    bool fastRun(const bool diagonal){
      if(!calcShortestDirs(diagonal)){
        printf("Failed to find shortest path!\n");
        return false;
      }
      /* move robot here */
      return true;
    }

  protected:
    virtual void waitForEndAction() {}
    virtual void queueAction(const Action action) {}
    virtual void findWall(bool& left, bool& front, bool& right, bool& back) {}
    virtual void backupMazeToFlash(){}
    virtual void stopDequeue() {}
    virtual void startDequeue() {}
    virtual void calibration() {}
    virtual void calcNextDirsPreCallback() {}
    virtual void calcNextDirsPostCallback(SearchAlgorithm::State prevState, SearchAlgorithm::State newState) {}
    virtual void discrepancyWithKnownWall() {}

  private:
    void turnbackSave(){
      queueAction(STOP_HALF);
      waitForEndAction();
      stopDequeue();
      backupMazeToFlash();
      queueAction(ROTATE_180);
      queueAction(STRAIGHT_HALF);
      startDequeue();
    }
    void queueNextDirs(const Dirs& nextDirs){
      for(const auto nextDir: nextDirs){
        const auto nextVec = getCurVec().next(nextDir);
        switch (Dir(nextDir - getCurDir())) {
          case Dir::Front: queueAction(STRAIGHT_FULL); break;
          case Dir::Left:  queueAction(TURN_LEFT_90);  break;
          case Dir::Right: queueAction(TURN_RIGHT_90); break;
          case Dir::Back:  turnbackSave();             break;
        }
        updateCurVecDir(nextVec, nextDir);
      }
    }
    bool generalSearchRun(){
      int cnt=0;
      while(1){
        // if(cnt++ > 300) return false;

        const auto& v = getCurVec();
        const auto& d = getCurDir();

        calcNextDirsPreCallback();
        auto prevState = getState();
        auto status = calcNextDirs(); //< 時間がかかる処理！
        auto newState = getState();
        calcNextDirsPostCallback(prevState, newState);

        // 既知区間移動をキューにつめる
        queueNextDirs(getNextDirs());

        if(status==SearchAlgorithm::Reached) return true;
        if(status==SearchAlgorithm::Error) return false;

        waitForEndAction();

        // 壁を確認
        bool left, front, right, back;
        findWall(left, front, right, back);
        if(!updateWall(v, d, left, front, right, back)){
          printf("There was a discrepancy with known information.\n");
          discrepancyWithKnownWall();
        }

        // 壁のない方向へ1マス移動
        Dir nextDir;
        if(!findNextDir(v, d, nextDir)){
          printInfo();
          printf("I can't go anywhere!\n");
          return false;
        }
        queueNextDirs({nextDir});
      }
    }
  };
}
