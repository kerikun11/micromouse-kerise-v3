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
	#define SEARCHING_ADDITIALLY_AT_START 0

	/** @class SearchAlgorithm
	*   @brief 迷路探索アルゴリズムを司るクラス
	*/
	class SearchAlgorithm {
	public:
		/** @brief コンストラクタ
		*   @param maze 使用する迷路の参照
		*   @param goal ゴール区画の配列
		*/
		SearchAlgorithm(Maze& maze, const std::vector<Vector>& goal) : maze(maze),
		stepMapGoal(maze), stepMapStart(maze), stepMapCandidates(maze),
		goal(goal) {}
		/** @enum State
		*   @brief 探索状態を列挙
		*/
		enum State{
			IDOLE,									//< 初期状態
			SEARCHING_FOR_GOAL,			//< ゴール区画を探索中
			REACHED_GOAL,           //< ゴール区画内を走行中
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
		void setGoal(const std::vector<Vector>& goal) {
			this->goal = goal;
		}
		/** @function calcNextDirs
		*   @brief 次に行くべき方向を計算する
		*   @param pv 出発位置
		*   @param pd 出発方向
		*   @param state 出発時の探索状態
		*/
		bool calcNextDirs(enum State& state, const Vector& pv, const Dir& pd, std::vector<Dir>& nextDirs, std::vector<Dir>& nextDirsInAdvance, const bool isForceBackToStart = false) {
			if(state == IDOLE){
				state = SEARCHING_FOR_GOAL;
				#if SEARCHING_ADDITIALLY_AT_START
				state = SEARCHING_ADDITIONALLY;
				#endif
				// 強制帰還がリクエストされていたらゴールを目指す
				if(isForceBackToStart) state = SEARCHING_FOR_GOAL;
			}

			if(state == SEARCHING_FOR_GOAL){
				// ゴール区画が探索済みなら次のstateへ
				const auto unknownGoal = std::find_if(goal.begin(), goal.end(), [&](const Vector& v){ return maze.unknownCount(v); });
				if(unknownGoal == goal.end()) state = SEARCHING_ADDITIONALLY;
				// ゴール区画かどうか判定
				if(std::find(goal.begin(), goal.end(), pv) != goal.end()){
					state = REACHED_GOAL;
					state = SEARCHING_ADDITIONALLY;
				}else{
					// ゴールを目指して探索
					stepMapGoal.update(goal, false, false);
					return calcNextDirsByStepMap(stepMapGoal, pv, pd, nextDirs, nextDirsInAdvance);
				}
			}

			if(state == REACHED_GOAL){
				// ゴール区画をすべて探索
				candidates.clear();
				for(const auto& v: goal) if(maze.unknownCount(v)) candidates.push_back(v);
				if(candidates.empty()){
					state = SEARCHING_ADDITIONALLY;
				}else{
					stepMapCandidates.update(candidates);
					return calcNextDirsByStepMap(stepMapCandidates, pv, pd, nextDirs, nextDirsInAdvance);
				}
			}

			if(state == SEARCHING_ADDITIONALLY){
				// 強制帰還がリクエストされていたら帰る
				if(isForceBackToStart) state = BACKING_TO_START;
				// 最短になりうる区画の洗い出し
				findShortestCandidates();
				if(candidates.empty()){
					state = BACKING_TO_START;
				}else{
					stepMapCandidates.update(candidates, false, false);
					return calcNextDirsByStepMap(stepMapCandidates, pv, pd, nextDirs, nextDirsInAdvance);
				}
			}

			if(state == BACKING_TO_START){
				if(pv == start) {
					state = REACHED_START;
				}else{
					stepMapStart.update({start}, false, false);
					return calcNextDirsByStepMap(stepMapStart, pv, pd, nextDirs, nextDirsInAdvance);
				}
			}

			return state == REACHED_START;
		}
		/** @function calcShortestDirs
		*   @brief 最短経路を導出
		*   @return 成功 or 失敗
		*/
		bool calcShortestDirs(std::vector<Dir>& shortestDirs, const bool diagonal = true){
			stepMapGoal.update(goal, true, diagonal);
			// stepMapGoal.update(goal, false, diagonal); //< for debug
			shortestDirs.clear();
			auto v = start;
			Dir dir = Dir::North;
			auto prev_dir = dir;
			while(1){
				step_t min_step = MAZE_STEP_MAX;
				const auto& dirs = dir.ordered(prev_dir);
				prev_dir = dir;
				for(const auto& d: dirs){
					if(!maze.canGo(v, d)) continue;
					step_t next_step = stepMapGoal.getStep(v.next(d));
					if(min_step > next_step) {
						min_step = next_step;
						dir = d;
					}
				}
				if(stepMapGoal.getStep(v) <= min_step) return false; //< 失敗
				shortestDirs.push_back(dir);
				v = v.next(dir);
				if(stepMapGoal.getStep(v) == 0) break; //< ゴール区画
			}
			// ゴール区画を行けるところまで直進する
			bool loop = true;
			while(loop){
				loop = false;
				std::vector<Dir> dirs;
				switch (Dir(dir-prev_dir)) {
					case Dir::Left: dirs = {dir.getRelative(Dir::Right), dir}; break;
					case Dir::Right: dirs = {dir.getRelative(Dir::Left), dir}; break;
					case Dir::Forward: default: dirs = {dir}; break;
				}
				if(!diagonal) dirs = {dir};
				for(const auto& d: dirs){
					if(maze.canGo(v, d)){
						shortestDirs.push_back(d);
						v = v.next(d);
						prev_dir = dir;
						dir = d;
						loop = true;
						break;
					}
				}
			}
			return true;
		}
		void printMap(const State& state, const Vector& v, const Dir& d) const {
			for(int i=0; i<MAZE_SIZE*2; i++) printf("\x1b[A");
			switch(state){
				case SearchAlgorithm::IDOLE:
				case SearchAlgorithm::SEARCHING_FOR_GOAL:
				stepMapGoal.print(v, d);
				break;
				case SearchAlgorithm::REACHED_GOAL:
				case SearchAlgorithm::SEARCHING_ADDITIONALLY:
				stepMapCandidates.print(v, d);
				break;
				case SearchAlgorithm::BACKING_TO_START:
				stepMapStart.print(v, d);
				break;
				case SearchAlgorithm::REACHED_START:
				case SearchAlgorithm::GOT_LOST:
				default:
				stepMapGoal.print(v, d);
				break;
			}
		}
	private:
		Maze& maze; /**< 使用する迷路の参照 */
		StepMap stepMapGoal; /**< 使用するステップマップ */
		StepMap stepMapStart; /**< 使用するステップマップ */
		StepMap stepMapCandidates; /**< 使用するステップマップ */
		const Vector start{0, 0}; /**< スタート区画を定義 */
		std::vector<Vector> goal; /**< ゴール区画を定義 */
		std::vector<Vector> candidates; /**< 最短経路上になり得る候補を入れるコンテナ */

		/** @function calcNextDirsByStepMap
		*   @brief ステップマップにより次に行くべき方向列を生成する
		*   @param stepMap ステップマップの選択
		*   @return true:成功, false:失敗(迷子)
		*/
		bool calcNextDirsByStepMap(StepMap& stepMap, const Vector& start_v, const Dir& start_d, std::vector<Dir>& nextDirs, std::vector<Dir>& nextDirsInAdvance) const {
			// ステップマップから既知区間方向列を生成
			nextDirs.clear();
			auto focus_v = start_v;
			auto dir = start_d;
			while(1){
				if(maze.unknownCount(focus_v)) break; //< 未知壁があれば，既知区間は終了
				// 周囲の区画のうち，最小ステップの方向を求める
				step_t min_step = MAZE_STEP_MAX;
				for(const auto& d: {dir+0, dir+1, dir-1, dir+2}){
					if(maze.isWall(focus_v, d)) continue;
					step_t next_step = stepMap.getStep(focus_v.next(d));
					if(min_step > next_step) {
						min_step = next_step;
						dir = d;
					}
				}
				if(stepMap.getStep(focus_v) <= min_step) break; //< 永遠ループ防止
				nextDirs.push_back(dir); //< 既知区間移動
				focus_v = focus_v.next(dir); //< 位置を更新
			}
			// ステップマップから未知壁方向の優先順位方向列を生成
			std::vector<Dir> dirs;
			// 方向の候補を抽出
			for(const auto& d: {dir+0, dir+1, dir-1, dir+2}) if(!maze.isWall(focus_v, d) && stepMap.getStep(focus_v.next(d))!=MAZE_STEP_MAX) dirs.push_back(d);
			// ステップが小さい順に並べ替え
			std::sort(dirs.begin(), dirs.end(), [&](const Dir& d1, const Dir& d2){
				if(maze.unknownCount(focus_v.next(d2))) return false; //< 未知壁優先
				return stepMap.getStep(focus_v.next(d1)) < stepMap.getStep(focus_v.next(d2)); //< 低コスト優先
			});
			nextDirsInAdvance = dirs;
			if(nextDirsInAdvance.empty()) return false;
			return true;
		}
		/** @function findShortestCandidates
		*   @brief ステップマップにより最短経路上になりうる区画を洗い出す
		*/
		bool findShortestCandidates(){
			candidates.clear();
			for(const bool diagonal: {true, false}){
				stepMapGoal.update(goal, false, diagonal);
				auto v = start;
				Dir dir = Dir::North;
				auto prev_dir = dir;
				while(1){
					step_t min_step = MAZE_STEP_MAX;
					const auto& dirs = dir.ordered(prev_dir);
					prev_dir = dir;
					for(const auto& d: dirs){
						if(maze.isWall(v, d)) continue;
						step_t next_step = stepMapGoal.getStep(v.next(d));
						if(min_step > next_step) {
							min_step = next_step;
							dir = d;
						}
					}
					if(stepMapGoal.getStep(v) <= min_step) return false; //< 失敗
					if(maze.unknownCount(v)) candidates.push_back(v);
					v = v.next(dir);
					if(stepMapGoal.getStep(v) == 0) break; //< ゴール区画
				}
				// ゴール区画を行けるところまで直進する
				bool loop = true;
				while(loop){
					loop = false;
					std::vector<Dir> dirs;
					switch (Dir(dir-prev_dir)) {
						case Dir::Left: dirs = {dir.getRelative(Dir::Right), dir}; break;
						case Dir::Right: dirs = {dir.getRelative(Dir::Left), dir}; break;
						case Dir::Forward: default: dirs = {dir}; break;
					}
					if(!diagonal) dirs = {dir};
					for(const auto& d: dirs){
						if(!maze.isWall(v, d)){
							if(maze.unknownCount(v)) candidates.push_back(v);
							v = v.next(d);
							prev_dir = dir;
							dir = d;
							loop = true;
							break;
						}
					}
				}
			}
			return true;
		}
	};
}
