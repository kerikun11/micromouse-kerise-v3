/**
* @file SearchAlgorithm.h
* @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
* @author KERI (Github: kerikun11)
* @url https://kerikeri.top/
* @date 2017.11.05
*/
#pragma once

#include "Maze.h"
#include "StepMap.h"

namespace MazeLib {
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
		*   @param goals ゴール区画の配列
		*/
		SearchAlgorithm(Maze& maze) : maze(maze) {}
		/** @enum Status
		*   @brief 進むべき方向の計算結果
		*/
		enum Status {
			Processing,
			Reached,
			Error,
		};
		/** @enum State
		*   @brief 探索状態を列挙
		*/
		enum State {
			START,									//< 初期位置，初期姿勢
			SEARCHING_FOR_GOAL,			//< ゴール区画を探索中
			SEARCHING_ADDITIONALLY,	//< 追加探索中
			BACKING_TO_START, 			//< スタートに戻っている
			REACHED_START,					//< スタートに戻ってきた
			IMPOSSIBLE,							//< ゴールにだどりつくことができないと判明した
			IDENTIFYING_POSITION,		//< 自己位置同定中
			GOING_TO_GOAL,          //< ゴールへ向かっている
		};
		/** @function stateString
		*   @brief Stateの表示用文字列を返す関数
		*/
		static const char* stateString(const enum State s)
		{
			static const char* str[]={
				"start",
				"Searching for Goal",
				"Searching Additionally",
				"Backing to Start",
				"Reached Start",
				"Impossible",
				"Identifying Position",
				"Going to Goal",
			};
			return str[s];
		}
		/** @function isComplete
		*   @brief 最短経路が導出されているか調べる関数
		*/
		bool isComplete()
		{
			Vectors candidates;
			findShortestCandidates(candidates);
			return candidates.empty();
		}
		void positionIdentifyingInit()
		{
			idStartVector = Vector(MAZE_SIZE/2, MAZE_SIZE/2);
			idMaze.reset(false);
		}
		bool updateWall(const State& state, const Vector& v, const Dir& d, const bool left, const bool front, const bool right, const bool back)
		{
			bool result = true;
			result = result & updateWall(state, v, d+1, left); // left wall
			result = result & updateWall(state, v, d+0, front); // front wall
			result = result & updateWall(state, v, d-1, right); // right wall
			result = result & updateWall(state, v, d+2, back); // back wall
			return result;
		}
		bool updateWall(const State& state, const Vector& v, const Dir& d, const bool& b)
		{
			if(state == IDENTIFYING_POSITION) return idMaze.updateWall(v, d, b);
			return maze.updateWall(v, d, b);
		}
		bool resetLastWall(const State& state, const int num = 1)
		{
			if(state == IDENTIFYING_POSITION) return maze.resetLastWall(num);
			return maze.resetLastWall(num);
		}
		Status calcNextDirs(State& state, Vector& curVec, const Dir& curDir, Dirs& nextDirs, Dirs& nextDirCandidates, bool& isPositionIdentifying, bool& isForceBackToStart, bool& isForceGoingToGoal, int& matchCount)
		{
			state = START;
			SearchAlgorithm::Status status;
			if(isPositionIdentifying){
				state = IDENTIFYING_POSITION;
				status = calcNextDirsPositionIdentification(curVec, curDir, nextDirs, nextDirCandidates, matchCount);
				switch(status){
					case SearchAlgorithm::Processing: return status;
					case SearchAlgorithm::Reached: isPositionIdentifying = false; break;
					case SearchAlgorithm::Error: return status;
				}
			}
			if(!SEARCHING_ADDITIALLY_AT_START){
				state = SEARCHING_FOR_GOAL;
				status = calcNextDirsSearchForGoal(curVec, curDir, nextDirs, nextDirCandidates);
				switch(status){
					case SearchAlgorithm::Processing: return status;
					case SearchAlgorithm::Reached: break;
					case SearchAlgorithm::Error: return status;
				}
			}
			if(!isForceBackToStart){
				state = SEARCHING_ADDITIONALLY;
				status = calcNextDirsSearchAdditionally(curVec, curDir, nextDirs, nextDirCandidates);
				switch(status){
					case SearchAlgorithm::Processing: return status;
					case SearchAlgorithm::Reached: break;
					case SearchAlgorithm::Error: return status;
				}
			}
			if(isForceGoingToGoal){
				state = GOING_TO_GOAL;
				status = calcNextDirsGoingToGoal(curVec, curDir, nextDirs, nextDirCandidates);
				switch(status){
					case SearchAlgorithm::Processing: return status;
					case SearchAlgorithm::Reached: isForceGoingToGoal = false; return SearchAlgorithm::Processing;
					case SearchAlgorithm::Error: return status;
				}
			}
			state = BACKING_TO_START;
			status = calcNextDirsBackingToStart(curVec, curDir, nextDirs, nextDirCandidates);
			switch(status){
				case SearchAlgorithm::Processing: return status;
				case SearchAlgorithm::Reached: break;
				case SearchAlgorithm::Error: return status;
			}
			state = REACHED_START;
			return status;
		}
		bool findNextDir(const State state, const Vector v, const Dir d, const Dirs& nextDirCandidates, Dir& nextDir) const
		{
			return findNextDir(state==IDENTIFYING_POSITION ? idMaze : maze, v, d, nextDirCandidates, nextDir);
		}
		bool findNextDir(const Maze& maze, const Vector v, const Dir d, const Dirs& nextDirCandidates, Dir& nextDir) const
		{
			// 候補の中で行ける方向を探す
			const auto it = std::find_if(nextDirCandidates.begin(), nextDirCandidates.end(), [&](const Dir& dir){
				return maze.canGo(v, dir);
			});
			if(it == nextDirCandidates.end()) return false;
			nextDir = *it;
			return true;
		}
		bool calcShortestDirs(Dirs& shortestDirs, const bool diagonal = true)
		{
			stepMap.update(maze, maze.getGoals(), true, diagonal);
			// update(maze, maze.getGoals(), false, diagonal); //< for debug
			shortestDirs.clear();
			auto v = maze.getStart();
			Dir dir = Dir::North;
			auto prev_dir = dir;
			while(1){
				step_t min_step = MAZE_STEP_MAX;
				// const auto& dirs = dir.ordered(prev_dir);
				// prev_dir = dir;
				// for(const auto& d: dirs){
				for(const auto d: Dir::All()) {
					if(!maze.canGo(v, d)) continue;
					step_t next_step = stepMap.getStep(v.next(d));
					if(min_step > next_step) {
						min_step = next_step;
						dir = d;
					}
				}
				if(stepMap.getStep(v) <= min_step) return false; //< 失敗
				shortestDirs.push_back(dir);
				v = v.next(dir);
				if(stepMap.getStep(v) == 0) break; //< ゴール区画
			}
			// ゴール区画を行けるところまで直進する
			bool loop = true;
			while(loop){
				loop = false;
				Dirs dirs;
				switch (Dir(dir-prev_dir)) {
					case Dir::Left: dirs = {Dir(dir+Dir::Right), dir}; break;
					case Dir::Right: dirs = {Dir(dir+Dir::Left), dir}; break;
					case Dir::Front: default: dirs = {dir}; break;
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
		void printMap(const State state, const Vector vec, const Dir dir) const
		{
			if(state == IDENTIFYING_POSITION) stepMap.print(idMaze, vec, dir);
			else stepMap.print(maze, vec, dir);
		}
		const StepMap& getStepMap() const { return stepMap; }
		const Maze& getMaze() const { return maze; }
		const Vector& getIdStartVector() const { return idStartVector; }
		void setIdStartVector(const Vector& v) { idStartVector=v; }

	protected:
		Maze& maze; /**< 使用する迷路の参照 */
		StepMap stepMap; /**< 使用するステップマップ */

	private:
		Maze idMaze; /**< 自己位置同定に使用する迷路 */
		Vector idStartVector;

		/** @function findShortestCandidates
		*   @brief ステップマップにより最短経路上になりうる区画を洗い出す
		*/
		bool findShortestCandidates(Vectors& candidates)
		{
			candidates.clear();
			// 斜めありなしの双方の最短経路上を候補とする
			for(const bool diagonal: {true, false}){
				stepMap.update(maze, maze.getGoals(), false, diagonal);
				auto v = maze.getStart();
				Dir dir = Dir::North;
				auto prev_dir = dir;
				while(1){
					step_t min_step = MAZE_STEP_MAX;
					// const auto& dirs = dir.ordered(prev_dir);
					// prev_dir = dir;
					// for(const auto& d: dirs){
					for(const auto d: Dir::All()) {
						if(maze.isWall(v, d)) continue;
						step_t next_step = stepMap.getStep(v.next(d));
						if(min_step > next_step) {
							min_step = next_step;
							dir = d;
						}
					}
					if(stepMap.getStep(v) <= min_step) return false; //< 失敗
					if(maze.unknownCount(v)) candidates.push_back(v); //< 未知壁があれば候補に入れる
					v = v.next(dir);
					if(stepMap.getStep(v) == 0) break; //< ゴール区画
				}
				// ゴール区画を行けるところまで直進する
				bool loop = true;
				while(loop){
					loop = false;
					Dirs dirs;
					switch (Dir(dir-prev_dir)) {
						case Dir::Left: dirs = {Dir(dir+Dir::Right), dir}; break;
						case Dir::Right: dirs = {Dir(dir+Dir::Left), dir}; break;
						case Dir::Front: default: dirs = {dir}; break;
					}
					if(!diagonal) dirs = {dir};
					for(const auto& d: dirs){
						if(!maze.isWall(v, d)){
							if(maze.unknownCount(v)) candidates.push_back(v); //< 未知壁があれば候補に入れる
							v = v.next(d);
							prev_dir = dir;
							dir = d;
							loop = true;
							break;
						}
					}
				}
			}
			return true; //< 成功
		}
		int countIdentityCandidates(const WallLogs idWallLogs, Vector& ans) const
		{
			int cnt = 0;
			for(int x=-MAZE_SIZE/2; x<MAZE_SIZE/2; x++)
			for(int y=-MAZE_SIZE/2; y<MAZE_SIZE/2; y++) {
				const Vector offset(x, y);
				int diffs=0;
				int matchs=0;
				int unknown=0;
				for(auto wl: idWallLogs){
					Vector v(wl.x, wl.y);
					Dir d = wl.d;
					if(maze.isKnown(v+offset, d) && maze.isWall(v+offset, d) != wl.b) diffs++;
					if(maze.isKnown(v+offset, d) && maze.isWall(v+offset, d) == wl.b) matchs++;
					if(!maze.isKnown(v+offset, d)) unknown++;
				}
				// int size = idWallLogs.size();
				if(diffs <= 4) {
					// if(size<4 || unknown<size/2 || matchs>size/2) {
					ans = idStartVector + offset;
					cnt++;
					// }
				}
			}
			return cnt;
		}
		enum Status calcNextDirsSearchForGoal(const Vector& cv, const Dir& cd, Dirs& nextDirsKnown, Dirs& nextDirCandidates)
		{
			Vectors candidates;
			for(auto v: maze.getGoals()) if(maze.unknownCount(v)) candidates.push_back(v); //< ゴール区画の未知区画を洗い出す
			if(candidates.empty()) return Reached;
			stepMap.calcNextDirs(maze, candidates, cv, cd, nextDirsKnown, nextDirCandidates);
			return nextDirCandidates.empty() ? Error : Processing;
		}
		enum Status calcNextDirsSearchAdditionally(const Vector& cv, const Dir& cd, Dirs& nextDirsKnown, Dirs& nextDirCandidates)
		{
			Vectors candidates;
			findShortestCandidates(candidates); //< 最短になりうる区画の洗い出し
			if(candidates.empty()) return Reached;
			stepMap.calcNextDirs(maze, candidates, cv, cd, nextDirsKnown, nextDirCandidates);
			return nextDirCandidates.empty() ? Error : Processing;
		}
		enum Status calcNextDirsBackingToStart(const Vector& cv, const Dir& cd, Dirs& nextDirsKnown, Dirs& nextDirCandidates)
		{
			const auto v = stepMap.calcNextDirs(maze, {maze.getStart()}, cv, cd, nextDirsKnown, nextDirCandidates);
			if(v == maze.getStart()) return Reached;
			return nextDirCandidates.empty() ? Error : Processing;
		}
		enum Status calcNextDirsGoingToGoal(const Vector& cv, const Dir& cd, Dirs& nextDirsKnown, Dirs& nextDirCandidates)
		{
			const auto goals = maze.getGoals();
			const auto v = stepMap.calcNextDirs(maze, goals, cv, cd, nextDirsKnown, nextDirCandidates);
			auto it = std::find_if(goals.begin(), goals.end(), [v](const Vector nv){ return v==nv; });
			if(it != goals.end()) return Reached;
			return nextDirCandidates.empty() ? Error : Processing;
		}
		enum Status calcNextDirsPositionIdentification(Vector& cv, const Dir& cd, Dirs& nextDirsKnown, Dirs& nextDirCandidates, int& matchCount)
		{
			Vector ans;
			int cnt = countIdentityCandidates(idMaze.getWallLogs(), ans);
			matchCount = cnt;
			if(cnt == 1) {
				cv = cv - idStartVector + ans;
				return Reached;
			} else if(cnt == 0){
				return Error;
			}
			Vectors candidates;
			for(int8_t x=0; x<MAZE_SIZE; ++x) for(int8_t y=0; y<MAZE_SIZE; ++y) if(idMaze.unknownCount(Vector(x,y))) candidates.push_back(Vector(x,y));
			stepMap.calcNextDirs(idMaze, candidates, cv, cd, nextDirsKnown, nextDirCandidates);
			return nextDirCandidates.empty() ? Error : Processing;
		}
	};
}
