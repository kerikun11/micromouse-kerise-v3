/**
* @file SearchAlgorithm.h
* @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
* @author KERI (Github: kerikun11)
* @date 2017.11.05
*/
#pragma once

#include "Maze.h"
#include "StepMap.h"

namespace MazeLib {
	/** @def FIND_ALL_WALL
	*   @brief 全探索するかどうか
	*   true: 全探索
	*   false: 最短になり得ないところは排除
	*/
	#define FIND_ALL_WALL 0
	/** @def SEARCHING_ADDITIALLY_AT_START
	*   @brief 追加探索状態で探索を始める(ゴールを急がない)
	*/
	#define SEARCHING_ADDITIALLY_AT_START 1

	/** @class SearchAlgorithm
	*   @brief 迷路探索アルゴリズムを司るクラス
	*/
	class SearchAlgorithm {
	public:
		/** @brief コンストラクタ
		*   @param maze 使用する迷路の参照
		*   @param goal ゴール区画の配列
		*/
		SearchAlgorithm(Maze& maze, const std::vector<Vector>& goal)
		: maze(maze), stepMapGoal(maze), stepMapStart(maze), stepMapCandidates(maze), goal(goal){ reset(); }
		/** @enum State
		*   @brief 探索状態を列挙
		*/
		enum State{
			IDOLE,									//< 初期状態
			SEARCHING_FOR_GOAL,			//< ゴール区画を探索中
			REACHED_GOAL,                        //< ゴール区画内を走行中
			SEARCHING_ADDITIONALLY,	//< 追加探索中
			BACKING_TO_START, 			//< スタートに戻っている
			REACHED_START,					//< スタートに戻ってきた
			FORCE_BACKING_TO_START,	//< 探索をやめてスタートに戻っている
			GOT_LOST,								//< ゴールにだどりつくことができないと判明した
		};
		/** @function stateString
		*   @brief SearchAlgorithm::Stateの表示用文字列を返す関数
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
			stepMapGoal.reset();
			stepMapStart.reset();
			stepMapCandidates.reset();
			curVec = Vector(0, 0);
			curDir = Dir::North;
			state = IDOLE;
			// calcNextDir();
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
		*   @return 成功 or 失敗
		*/
		bool calcShortestDirs(){
			// stepMapGoal.update(goal, true);
			stepMapGoal.update(goal, false); //< for debug
			shortestDirs.clear();
			auto v = start;
			auto dir = Dir(Dir::North);
			auto prev_dir = dir;
			while(1){
				prev_dir = dir;
				step_t min_step = MAZE_STEP_MAX;
				for(const auto& d: Dir::All()){
					if(!maze.canGo(v, d)) continue;
					step_t next_step = stepMapGoal.getStep(v.next(d));
					if(min_step > next_step) {
						min_step = next_step;
						dir = d;
					}
				}
				if(stepMapGoal.getStep(v) <= min_step) return false;
				shortestDirs.push_back(dir);
				v = v.next(dir);
				if(stepMapGoal.getStep(v) == 0) break; //< ゴール区画
			}
			// ゴール区画を行けるところまで直進する
			bool loop = true;
			while(loop){
				loop = false;
				std::vector<Dir> dirs;
				if(Dir(dir-prev_dir)==Dir::Left) dirs={Dir(dir+3), dir};
				else if(Dir(dir-prev_dir)==Dir::Right) dirs={Dir(dir+1), dir};
				else dirs={dir};
				for(auto& d: dirs){
					if(maze.canGo(v, d)){
						shortestDirs.push_back(d);
						v=v.next(d);
						prev_dir = dir;
						dir = d;
						loop = true;
					}
				}
			}
			return true;
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
					stepMapGoal.print(curVec, curDir);
					break;
					case REACHED_GOAL:
					case SEARCHING_ADDITIONALLY:
					stepMapCandidates.print(curVec, curDir);
					break;
					case BACKING_TO_START:
					stepMapStart.print(curVec, curDir);
					break;
					case REACHED_START:
					case GOT_LOST:
					default:
					stepMapGoal.print(curVec, curDir);
					break;
				}
			}
			printf("Cur: ( %3d, %3d, %3d), State: %s       \n", curVec.x, curVec.y, uint8_t(curDir), stateString(state));
			printf("Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d, Known: %3d\n", step, f, l, r, b, k);
		}
		/** @function printPath
		*   @brief 最短経路の表示
		*/
		void printPath() const {
			//for(int i=0; i<MAZE_SIZE*2+5; i++) printf("\x1b[A");
			maze.printPath(Vector(0, 0), shortestDirs);
			printf("Shortest Step: %d\n", shortestDirs.size()-1);
		}
	private:
		State state; /**< 現在の探索状態を保持 */
		Maze& maze; /**< 使用する迷路の参照 */
		StepMap stepMapGoal; /**< 使用するステップマップ */
		StepMap stepMapStart; /**< 使用するステップマップ */
		StepMap stepMapCandidates; /**< 使用するステップマップ */
		const Vector start{0, 0}; /**< スタート区画を定義 */
		std::vector<Vector> goal; /**< ゴール区画を定義 */
		Vector curVec; /**< 現在の区画座標 */
		Dir curDir; /**< 現在向いている方向 */
		std::vector<Dir> nextDirs; /**< 次に行くべき探索方向配列 */
		std::vector<Dir> nextDirInAdvance; /**< 最短経路の方向配列 */
		std::vector<Dir> shortestDirs; /**< 最短経路の方向配列 */
		std::vector<Vector> candidates; /**< 最短経路上になり得る候補を入れるコンテナ */
		int step=0,f=0,l=0,r=0,b=0,k=0; /**< 探索の評価のためのカウンタ */

		/** @function calcNextDirByStepMap
		*   @brief ステップマップにより次に行くべき方向列を生成する
		*   @param stepMap ステップマップの選択
		*   @return true:成功, false:失敗(迷子)
		*/
		const std::vector<Dir> calcNextDirByStepMap(StepMap& stepMap, Vector focus_v, Dir focus_d){
			std::vector<Dir> nextDirs;
			while(1){
				step_t min_step = MAZE_STEP_MAX;
				for(const auto& d: Dir::All()){
					if(maze.isWall(focus_v, d)) continue;
					step_t next_step = stepMap.getStep(focus_v.next(d));
					if(min_step > next_step) {
						min_step = next_step;
						focus_d = d;
					}
				}
				if(!maze.isKnown(focus_v, focus_d)) break;
				if(stepMap.getStep(focus_v) <= min_step) break;
				nextDirs.push_back(focus_d);
				focus_v = focus_v.next(focus_d);
			}
			return nextDirs;
		}
		/** @function findShortestCandidates
		*   @brief ステップマップにより最短経路上になりうる区画を洗い出す
		*/
		void findShortestCandidates(){
			#if FIND_ALL_WALL
			stepMapGoal.update(goal);
			candidates.clear();
			for(int i=0; i<MAZE_SIZE; i++){
				for(int j=0; j<MAZE_SIZE; j++){
					Vector v(i,j);
					if(stepMapGoal.getStep(i, j) != MAZE_STEP_MAX && maze.unknownCount(v)){
						candidates.push_back(v);
					}
				}
			}
			#else
			stepMapStart.update({start});
			stepMapGoal.update(goal);
			candidates.clear();
			std::vector<step_t> goal_steps;
			for(const auto& g: goal) goal_steps.push_back(stepMapStart.getStep(g));
			step_t goal_step = *(std::min_element(goal_steps.begin(), goal_steps.end()));
			for(int i=0; i<MAZE_SIZE; i++){
				for(int j=0; j<MAZE_SIZE; j++){
					Vector v(i,j);
					// if(stepMapGoal.getStep(i, j) + stepMapStart.getStep(i, j) <= goal_step && maze.unknownCount(v)){
					if(stepMapGoal.getStep(i, j) + stepMapStart.getStep(i, j) <= goal_step+stepMapStart.extra && maze.unknownCount(v)){
						// if(stepMapGoal.getStep(i, j) + stepMapStart.getStep(i, j) <= goal_step+MAZE_SIZE && maze.unknownCount(v)){
						candidates.push_back(v);
					}
				}
			}
			#endif
		}
		/** @function calcNextDir
		*   @brief 次に行くべき方向を計算する
		*   @param pv 出発位置
		*   @param pd 出発方向
		*   @param state 出発時の探索状態
		*   @return 計算後の探索状態
		*   計算結果はメンバ変数のnextDirsに保存される．
		*/
		enum State calcNextDir(const Vector& pv, const Dir& pd, enum State state){
			if(state == IDOLE){
				state = SEARCHING_FOR_GOAL;
				// ゴール区画が探索済みなら次のstateへ
				if(std::find_if(goal.begin(), goal.end(), [&](const Vector& v){ return maze.unknownCount(v); }) == goal.end()) state = SEARCHING_ADDITIONALLY;
				#if SEARCHING_ADDITIALLY_AT_START
				state = SEARCHING_ADDITIONALLY;
				#endif
			}

			if(state == SEARCHING_FOR_GOAL){
				// ゴール区画かどうか判定
				if(std::find(goal.begin(), goal.end(), pv) != goal.end()){
					state = REACHED_GOAL;
					state = SEARCHING_ADDITIONALLY;
				}else{
					// stepMapGoal.updateSimple(goal);
					stepMapGoal.update(goal);
					nextDirs = calcNextDirByStepMap(stepMapGoal, pv, pd);
					if(nextDirs.empty()) return GOT_LOST;
				}
			}

			if(state == REACHED_GOAL){
				candidates.clear();
				for(const auto& v: goal) if(maze.unknownCount(v)) candidates.push_back(v);
				if(candidates.empty()){
					state = SEARCHING_ADDITIONALLY;
				}else{
					// stepMapCandidates.updateSimple(candidates);
					stepMapCandidates.update(candidates);
					nextDirs = calcNextDirByStepMap(stepMapCandidates, pv, pd);
					if(nextDirs.empty()) return GOT_LOST;
				}
			}

			if(state == SEARCHING_ADDITIONALLY){
				// 最短になりうる区画の洗い出し
				findShortestCandidates();
				if(candidates.empty()){
					state = BACKING_TO_START;
				}else{
					// stepMapCandidates.updateSimple(candidates);
					stepMapCandidates.update(candidates);
					nextDirs = calcNextDirByStepMap(stepMapCandidates, pv, pd);
					if(nextDirs.empty()) return GOT_LOST;
				}
			}

			if(state == BACKING_TO_START){
				if(pv == start) {
					state = REACHED_START;
				}else{
					stepMapStart.update({start});
					nextDirs = calcNextDirByStepMap(stepMapStart, pv, pd);
					if(nextDirs.empty()) return GOT_LOST;
				}
			}

			if(state == FORCE_BACKING_TO_START){
				if(pv == start) {
					state = REACHED_START;
				}else{
					stepMapStart.update({start}, true);
					nextDirs = calcNextDirByStepMap(stepMapStart, pv, pd);
					if(nextDirs.empty()) return GOT_LOST;
				}
			}

			for(const auto& d: nextDirs){
				f += pd.getRelative(Dir::Forward) == d;
				l += pd.getRelative(Dir::Left   ) == d;
				r += pd.getRelative(Dir::Right  ) == d;
				b += pd.getRelative(Dir::Back   ) == d;
			}
			step += nextDirs.size();
			k += nextDirs.size()-1;
			return state;
		}
	};
}
