/**
* @file Agent.h
* @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
* @author KERI (Github: kerikun11)
* @date 2017.11.05
*/
#pragma once

#include "Maze.h"
#include "StepMap.h"

/** @def DEEPNESS
*   @brief 探索の深さ
*   0: 歩数最短になり得ないところは排除
*   1: 全探索
*/
#define DEEPNESS 0
/** @def SEARCHING_ADDITIALLY_AT_START
*   @brief 追加探索状態で探索を始める(ゴールを急がない)
*/
#define SEARCHING_ADDITIALLY_AT_START 0

/** @class Agent
*   @brief 迷路探索アルゴリズムを司るクラス
*/
class Agent{
public:
  /** @brief コンストラクタ
  *   @param maze 使用する迷路の参照
  *   @param goal ゴール区画の配列
  */
  Agent(Maze& maze, const std::vector<Vector>& goal) : maze(maze), stepMap(maze), goal(goal){ reset(); }
  /** @enum State
  *   @brief 探索状態を列挙
  */
  enum State{
    IDOLE,                  //< 初期状態
    SEARCHING_FOR_GOAL,     //< ゴール区画を探索中
    REACHED_GOAL,           //< ゴール区画内を走行中
    SEARCHING_ADDITIONALLY, //< 追加探索中
    BACKING_TO_START,       //< スタートに戻っている
    REACHED_START,          //< スタートに戻ってきた
    FORCE_BACKING_TO_START, //< 探索をやめてスタートに戻っている
    GOT_LOST,               //< ゴールにだどりつくことができないと判明した
  };
  /** @function stateString
  *   @brief Agent::Stateの表示用文字列を返す関数
  */
  static const char* stateString(const enum State s){
    static const char* str[]={
      "idole",
      "Searching for Goal",
      "Reached Goal",
      "Searching Additionally",
      "Backing to Start",
      "Reached Start",
      "Force Backing to Start",
      "Got Lost",
    };
    return str[s];
  }
  /** @function reset
  *   @brief 探索状態をリセットして，現在地をスタート区画にセット
  *   迷路情報はそのまま保持
  */
  void reset(){
    stepMap.reset();
    curVec = Vector(0, 0);
    curDir = Dir::North;
    state = IDOLE;
    calcNextDir();
  }
  /** @function reset
  *   @brief ゴール区画を更新し，探索状態をリセットして，現在地をスタート区画にセット
  *   @param goal ゴール区画の配列
  *   迷路情報はそのまま保持
  */
  void reset(const std::vector<Vector>& goal) {
    this->goal = goal;
    reset();
  }
  /** @function forceBackToStart
  *   @brief 探索を中止してスタート区画へ強制的に戻る
  *   時間が残りわずかな時などに使う
  */
  void forceBackToStart(){
    if(state != REACHED_START) state = FORCE_BACKING_TO_START;
  }
  /** @function updateCurVecDir
  *   @brief 現在地を更新
  *   @param v 区画座標
  *   @param d 絶対方向
  */
  void updateCurVecDir(const Vector& v, const Dir& d){ curVec = v; curDir=d; }
  /** @function updateWall
  *   @brief 絶対座標絶対方向で壁の1枚更新
  *   @param v 区画座標
  *   @param d 絶対方向
  *   @param b 壁の有無
  */
  void updateWall(const Vector& v, const Dir& d, const bool& b){ maze.updateWall(v, d, b); }

  /** @function calcNextDir
  *   @brief 次に行くべき方向配列を計算
  *   注意: 処理に時間がかかる場合あり
  *   @return 探索状態
  */
  bool calcNextDir(){
    state = calcNextDir(curVec, curDir, state);
    return state != GOT_LOST;
  }
  /** @function calcShortestDirs
  *   @brief 最短経路を導出
  *   @return 最短経路の方向配列
  */
  bool calcShortestDirs(){
    stepMap.update(goal, StepMap::Goal, true);
    shortestDirs.clear();
    Vector v = start;
    Dir dir = Dir::North;
    Dir prev_dir = Dir::North;
    while(1){
      std::vector<Dir> dirs;
      if(Dir(dir-prev_dir)==Dir::Left) dirs={Dir(dir+3), dir, Dir(dir+1)};
      else if(Dir(dir-prev_dir)==Dir::Right) dirs={Dir(dir+1), dir, Dir(dir+3)};
      else dirs={dir, Dir(dir+1), Dir(dir+3)};
      auto it = std::find_if(dirs.begin(), dirs.end(),[&](const Dir& d){
        if(!maze.canGo(v, d)) return false;
        return stepMap.getStep(v.next(d)) == stepMap.getStep(v)-1;
      });
      if(it == dirs.end()) return false;
      prev_dir = dir;
      dir = *it;
      v=v.next(dir);
      shortestDirs.push_back(dir);
      if(stepMap.getStep(v)==0) break;
    }
    while(maze.canGo(v, dir)){
      shortestDirs.push_back(dir);
      v=v.next(dir);
    }
    return true;
  }
  /** @function calcNextDirInAdvance
  *   @brief 壁を見る前に前もって行くべき方向を計算
  *   取り得るすべての壁パターンについて計算
  *   開発中
  */
  const std::vector<std::vector<Dir>> calcNextDirInAdvance(){
    calcNextDir();
    std::vector<std::vector<Dir>> nextDirss;
    while(1){
      auto dirs = curDir.ordered();
      auto it = std::find_if(dirs.begin(), dirs.end(),[&](const Dir& d){
        if(maze.isWall(curVec, d)) return false;
        return stepMap.getStep(curVec.next(d)) == stepMap.getStep(curVec)-1;
      });
      if(it == dirs.end()) break;
      printf("curVec:(%d,%d), curDir:%d, *it:%d\n", curVec.x, curVec.y, int8_t(curDir), int8_t(*it));
      if(maze.isKnown(curVec, *it)){
        printf("isKnown; break;\n");
        calcNextDir();
        nextDirss.push_back(getNextDirs());
        break;
      }
      maze.setKnown(curVec, *it, true);
      if(calcNextDir(curVec, curDir, state)==GOT_LOST){
        //printInfo();
        break;
      }
      maze.setKnown(curVec, *it, false);
      nextDirss.push_back(getNextDirs());
      maze.setWall(curVec, *it, true);
    }
    for(auto d: Dir::All()) if(!maze.isKnown(curVec, d)) maze.setWall(curVec, d, false);
    calcNextDir();
    for(auto nd: nextDirss) {
      printf(">");
      for(auto d: nd) printf("%d ", int8_t(d));
      printf("\n");
    }
    return nextDirss;
  }

  /** @function getState
  *   @brief 探索状態の取得
  */
  const State& getState() const {
    return state;
  }
  /** @function getMaze
  *   @brief 迷路の取得
  */
  const Maze& getMaze() const {
    return maze;
  }
  /** @function getNextDirs
  *   @brief 次に行くべき方向配列の計算結果を取得
  */
  const std::vector<Dir>& getNextDirs() const {
    return nextDirs;
  }
  /** @function getCurVec
  *   @brief 現在区画を取得
  */
  const Vector& getCurVec() const {
    return curVec;
  }
  /** @function getCurDir
  *   @brief 現在の方向を取得
  */
  const Dir& getCurDir() const {
    return curDir;
  }
  /** @function getNextDirs
  *   @brief 最短経路の方向配列の計算結果を取得
  */
  const std::vector<Dir>& getShortestDirs() const {
    return shortestDirs;
  }
  /** @function printInfo
  *   @brief 探索状態を表示
  *   @param showMaze true:迷路も表示, false:迷路は非表示
  */
  void printInfo(const bool& showMaze = true) const {
    if(showMaze){
      for(int i=0; i<MAZE_SIZE*2+4; i++) printf("\x1b[A");
      switch(state){
        case IDOLE:
        case SEARCHING_FOR_GOAL:
        stepMap.print(StepMap::Goal, curVec, curDir);
        break;
        case REACHED_GOAL:
        case SEARCHING_ADDITIONALLY:
        stepMap.print(StepMap::General, curVec, curDir);
        break;
        case BACKING_TO_START:
        stepMap.print(StepMap::Start, curVec, curDir);
        break;
        case REACHED_START:
        case GOT_LOST:
        default:
        stepMap.print(StepMap::Goal, curVec, curDir);
        break;
      }
    }
    printf("Cur: ( %3d, %3d, %3d), State: %s       \n", curVec.x, curVec.y, uint8_t(curDir), stateString(state));
    printf("Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d\n", step, f, l, r, b);
  }
  /** @function printPath
  *   @brief 最短経路の表示
  */
  void printPath() const {
    //for(int i=0; i<MAZE_SIZE*2+5; i++) printf("\x1b[A");
    maze.printPath(Vector(0, 0), shortestDirs);
    printf("\n\n");
    printf("Shortest Step: %d\n", shortestDirs.size()-1);
  }
private:
  State state; /**< 現在の探索状態を保持 */
  Maze& maze; /**< 使用する迷路の参照 */
  StepMap stepMap; /**< 使用するステップマップ */
  const Vector start{0, 0}; /**< スタート区画を定義 */
  std::vector<Vector> goal; /**< ゴール区画を定義 */
  Vector curVec; /**< 現在の区画座標 */
  Dir curDir; /**< 現在向いている方向 */
  std::vector<Dir> nextDirs; /**< 次に行くべき探索方向配列 */
  std::vector<Dir> shortestDirs; /**< 最短経路の方向配列 */
  std::vector<Vector> candidates; /**< 最短経路上になり得る候補を入れるコンテナ */
  int step=0,f=0,l=0,r=0,b=0; /**< 探索の評価のためのカウンタ */

  /** @function calcNextDirByStepMap
  *   @brief ステップマップにより次に行くべき方向列を生成する
  *   @param sp ステップマップの選択
  *   @return true:成功, false:失敗(迷子)
  */
  bool calcNextDirByStepMap(const enum StepMap::Purpose& sp){
    nextDirs.clear();
    Vector focus_v = curVec;
    Dir focus_d = curDir;
    while(1){
      auto dirs = focus_d.ordered();
      auto it = std::find_if(dirs.begin(), dirs.end(), [&](const Dir& d){
        if(!maze.canGo(focus_v, d)) return false;
        return stepMap.getStep(focus_v.next(d), sp) == stepMap.getStep(focus_v, sp)-1;
      });
      if(it==dirs.end()) break;
      nextDirs.push_back(*it);
      focus_d = *it;
      focus_v = focus_v.next(*it);
    }
    if(nextDirs.empty()){
      return false;
    }
    return true;
  }
  /** @function findShortestCandidates
  *   @brief ステップマップにより最短経路上になりうる区画を洗い出す
  */
  void findShortestCandidates(){
    stepMap.update(goal, StepMap::Goal);
    stepMap.update({start}, StepMap::Start);
    candidates.clear();
    std::vector<step_t> goal_steps;
    for(auto g:goal) goal_steps.push_back(stepMap.getStep(g, StepMap::Start));
    step_t goal_step = *(std::min_element(goal_steps.begin(), goal_steps.end()));
    for(int i=0; i<MAZE_SIZE; i++){
      for(int j=0; j<MAZE_SIZE; j++){
        Vector v(i,j);
        #if DEEPNESS == 0
        if(stepMap.getStep(i,j) + stepMap.getStep(i,j,StepMap::Start) <= 1+goal_step && maze.knownCount(Vector(i,j))!=4){
          candidates.push_back(v);
        }
        #elif DEEPNESS == 1
        if(stepMap.getStep(i,j) != MAZE_STEP_MAX && maze.knownCount(Vector(i,j))!=4){
          candidates.push_back(v);
        }
        #endif
      }
    }
  }
  /** @function calcNextDir
  *   @brief 次に行くべき方向を計算する
  *   @param pv 出発位置
  *   @param pd 出発方向
  *   @param state 出発時の探索状態
  *   @return 計算後の探索状態
  *   計算結果はメンバ変数のnextDirsに保存される．
  */
  const enum State calcNextDir(const Vector& pv, const Dir& pd, enum State state){
    if(state == IDOLE){
      step=0; f=0; l=0; r=0; b=0;
      findShortestCandidates();
      if(candidates.empty()) state = BACKING_TO_START;
      else state = SEARCHING_FOR_GOAL;
      #if SEARCHING_ADDITIALLY_AT_START
      state = SEARCHING_ADDITIONALLY;
      #endif
    }

    if(state == SEARCHING_FOR_GOAL){
      if(std::find(goal.begin(), goal.end(), pv)!=goal.end()){
        state = REACHED_GOAL;
      }else{
        stepMap.update(goal, StepMap::Goal);
        if(!calcNextDirByStepMap(StepMap::Goal)) return GOT_LOST;
      }
    }

    if(state == REACHED_GOAL){
      candidates.clear();
      for(auto v: goal){
          if(maze.knownCount(v)!=4){
            candidates.push_back(v);
          }
      }
      if(candidates.empty()){
        state = SEARCHING_ADDITIONALLY;
      }else{
        stepMap.update(candidates, StepMap::General);
        if(!calcNextDirByStepMap(StepMap::General)) return GOT_LOST;
      }
    }


    if(state == SEARCHING_ADDITIONALLY){
      findShortestCandidates();
      if(candidates.empty()){
        state = BACKING_TO_START;
      }else{
        stepMap.update(candidates, StepMap::General);
        if(!calcNextDirByStepMap(StepMap::General)) return GOT_LOST;
      }
    }

    if(state == BACKING_TO_START){
      if(pv == start) {
        state = REACHED_START;
      }else{
        stepMap.update({start}, StepMap::Start);
        if(!calcNextDirByStepMap(StepMap::Start)) return GOT_LOST;
      }
    }

    if(state == FORCE_BACKING_TO_START){
      if(pv == start) {
        state = REACHED_START;
      }else{
        stepMap.update({start}, StepMap::Start, true);
        if(!calcNextDirByStepMap(StepMap::Start)) return GOT_LOST;
      }
    }

    for(auto d: nextDirs){
      step++;
      f += pd.getRelative(Dir::Forward) == d;
      l += pd.getRelative(Dir::Left   ) == d;
      r += pd.getRelative(Dir::Right  ) == d;
      b += pd.getRelative(Dir::Back   ) == d;
    }
    return state;
  }
};

