/**
* @file Agent.h
* @brief マイクロマウスの探索ナビゲータ
* @author KERI (Github: kerikun11)
* @url https://kerikeri.top/
* @date 2018.05.20
*/
#pragma once

#include "Maze.h"
#include "StepMap.h"
#include "SearchAlgorithm.h"

namespace MazeLib {
  class Agent {
  public:
    Agent(const Vectors& goals)
    : maze(goals), searchAlgorithm(maze) {}
		/** @function replaceGoals
		*   @brief ゴール区画を変更する関数
		*/
		void replaceGoals(const Vectors& goals)
		{ maze.setGoals(goals); }
    /** @function isComplete
    *   @brief 探索が完了しているかどうかを返す関数
    */
    bool isComplete()
    { return searchAlgorithm.isComplete(); }
    /** @function updateCurVecDir
    *   @brief 現在地を更新
    *   @param v 区画座標
    *   @param d 絶対方向
    */
    void updateCurVecDir(const Vector& v, const Dir& d)
    { curVec = v; curDir = d; }
    /** @function findNextDir
    *   @brief 次に行くべき方向を取得する
    */
    bool findNextDir(const Vector v, const Dir d, Dir& nextDir) const
    { return searchAlgorithm.findNextDir(state, v, d, nextDirCandidates, nextDir); }
    /** @function updateWall
    *   @brief 絶対座標絶対方向で壁の1枚更新
    *   @param v 区画座標
    *   @param d 絶対方向
    *   @param b 壁の有無
    */
    bool updateWall(const Vector& v, const Dir& d, const bool left, const bool front, const bool right, const bool back)
    { return searchAlgorithm.updateWall(state, v, d, left, front, right, back); }
    bool resetLastWall(const int num = 1)
    { return searchAlgorithm.resetLastWall(state, num); }
    /** @function calcNextDirs
    *   @brief 次に行くべき方向配列を計算
    *   注意: 処理に時間がかかる場合あり
    *   @return 探索状態
    */
    SearchAlgorithm::Status calcNextDirs()
    { return searchAlgorithm.calcNextDirs(state, curVec, curDir, nextDirsKnown, nextDirCandidates, isPositionIdentifying, isForceBackToStart, isForceGoingToGoal, matchCount); }
    /** @function calcShortestDirs
    *   @brief 最短経路を導出
    *   @param diagonal true: 斜めあり, false: 斜めなし
    *   @return true: 成功, false: 失敗
    */
    bool calcShortestDirs(const bool diagonal = true)
    { return searchAlgorithm.calcShortestDirs(shortestDirs, diagonal); }
    /** @function forceBackToStart
    *   @brief 探索を中止してスタート区画へ強制的に戻る
    *   時間が残りわずかな時などに使う
    */
    void forceBackToStart()
    {
      isForceBackToStart = true;
    }
    void forceGoingToGoal()
    {
      isForceGoingToGoal = true;
    }
    void positionIdentify(const Dir d = Dir::North)
    {
      searchAlgorithm.positionIdentifyingInit();
      isPositionIdentifying = true;
      state = SearchAlgorithm::IDENTIFYING_POSITION;
      curVec = searchAlgorithm.getIdStartVector();
      curDir = d;
    }
    /** @function getState
    *   @brief 探索状態の取得
    */
    const SearchAlgorithm::State& getState() const
    { return state; }
    /** @function getNextDirs
    *   @brief 次に行くべき方向配列の計算結果を取得
    */
    const Dirs& getNextDirs() const
    { return nextDirsKnown; }
    /** @function getNextDirs
    *   @brief 次に行くべき方向配列の計算結果を取得
    */
    const Dirs& getNextDirCandidates() const
    { return nextDirCandidates; }
    /** @function getCurVec
    *   @brief 現在区画を取得
    */
    const Vector& getCurVec() const
    { return curVec; }
    /** @function getCurDir
    *   @brief 現在の方向を取得
    */
    const Dir& getCurDir() const
    { return curDir; }
    /** @function getNextDirs
    *   @brief 最短経路の方向配列の計算結果を取得
    */
    const Dirs& getShortestDirs() const
    { return shortestDirs; }
    /** @function getMaze
    *   @brief 迷路を取得
    */
    Maze& getMaze()
    { return maze; }
    /** @function printInfo
    *   @brief 探索状態を表示
    *   @param showMaze true:迷路も表示, false:迷路は非表示
    */
    void printInfo(const bool showMaze = true) const
    { printInfo(showMaze, curVec, curDir, state); }
    void printInfo(const bool showMaze, const Vector vec, const Dir dir, const SearchAlgorithm::State state) const
    {
      // 迷路を表示
      if(showMaze) {
        printf(ESC_UP(10)); //< カーソルを移動
        printf(ESC_UP(32)); //< カーソルを移動
        printf(ESC_UP(32)); //< カーソルを移動
        searchAlgorithm.printMap(state, vec, dir);
      }
      // 詳細を表示
      printf("Cur: ( %2d, %2d,  %c), State: %s               \n", vec.x, vec.y, ">^<v"[dir], SearchAlgorithm::stateString(state));
      printf("nextDirsKnown: ");
      for (const auto d : getNextDirs()) printf("%c", ">^<v"[d]);
      printf("                         \n");
      printf("nextDirCandidates: ");
      for(const auto d: getNextDirCandidates()) printf("%c", ">^<v"[d]);
      printf("        \n");
      printf("Match Count: %d   \n", matchCount);
    }
    /** @function printPath
    *   @brief 最短経路の表示
    */
    void printPath() const
    {
      maze.printPath(maze.getStart(), shortestDirs);
      printf("Shortest Step: %d\n", (int)shortestDirs.size());
    }

  protected:
    Maze maze; /**< 使用する迷路 */
    SearchAlgorithm::State state; /**< 現在の探索状態を保持 */
    Vector curVec; /**< 現在の区画座標 */
    Dir curDir; /**< 現在向いている方向 */
    bool isForceBackToStart = false;
    bool isForceGoingToGoal = false;
    bool isPositionIdentifying = false;

  private:
    SearchAlgorithm searchAlgorithm; /**< 探索器 */
    Dirs nextDirsKnown; /**< 次に行く探索方向配列 */
    Dirs nextDirCandidates; /**< 次に行く方向の候補の優先順 */
    Dirs shortestDirs; /**< 最短経路の方向配列 */
    int matchCount = 0;
  };
}
