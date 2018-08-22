/**
* @file StepMap.h
* @brief マイクロマウスの迷路のステップマップを扱うクラス
* @author KERI (Github: kerikun11)
* @url https://kerikeri.top/
* @date 2017.11.05
*/
#pragma once

// inc
#include "Maze.h"

// src
#include <complex> // for std::sqrt()
#include <algorithm>
#include <queue>

namespace MazeLib {
	/** @def MAZE_STEP_MAX
	*  @brief ステップマップの最大値
	*/
	#define MAZE_STEP_MAX  9999
	typedef uint16_t step_t; /**< @brief ステップマップの型 */

	/** @class StepMap
	*  @brief 足立法のためのステップマップを管理するクラス
	*/
	class StepMap {
	public:
		/** @brief コンストラクタ
		*  @param maze 使用する迷路の参照
		*/
		StepMap()
		{
			calcStraightStepTable();
			reset();
		}
		/** @function reset
		*  @brief ステップマップの初期化
		*/
		void reset(const step_t step = MAZE_STEP_MAX)
		{
			for(int8_t y=0; y<MAZE_SIZE; y++)
			for(int8_t x=0; x<MAZE_SIZE; x++)
			setStep(x, y, step); //< ステップをクリア
		}
		/** @function getStep
		*  @param ステップへの参照の取得，書き込み可能
		*  @param v 区画の座標
		*  @return ステップメモリの参照
		*/
		const step_t& getStep(const Vector& v) const { return getStep(v.x, v.y); }
		const step_t& getStep(const int8_t& x, const int8_t& y) const
		{
			// (x, y) がフィールド内か確認
			if(x<0 || y<0 || x>MAZE_SIZE-1 || y>MAZE_SIZE-1){
				printf("Warning: refered to out of field ------------------------------------------> %2d, %2d\n", x, y);
				static step_t outside;  //< フィールド外のときの戻りメモリ
				outside = MAZE_STEP_MAX; //< フィールド外なら最大ステップとする
				return outside;
			}
			return stepMap[y][x];
		}
		/** @function getStep
		*  @param ステップへの参照の取得，書き込み可能
		*  @param v 区画の座標
		*  @return ステップメモリの参照
		*/
		bool setStep(const Vector& v, const step_t& step) { return setStep(v.x, v.y, step); }
		bool setStep(const int8_t& x, const int8_t& y, const step_t& step)
		{
			// (x, y) がフィールド内か確認
			if(x<0 || y<0 || x>=MAZE_SIZE || y>=MAZE_SIZE){
				printf("Warning: refered to out of field ------------------------------------------> %2d, %2d\n", x, y);
				return false;
			}
			stepMap[y][x] = step;
			return true;
		}
		/** @function print
		*  @param v ハイライト区画
		*/
		void print(const Maze& maze, const Vector& v = Vector(-1,-1), const Dir& d = Dir::AbsMax) const
		{
			print(std::cout, maze, v, d);
		}
		void print(std::ostream& os, const Maze& maze, const Vector& v = Vector(-1,-1), const Dir& d = Dir::AbsMax) const
		{
			os << std::endl;
			for(int8_t y=MAZE_SIZE; y>=0; y--){
				if(y != MAZE_SIZE){
					os << '|';
					for(uint8_t x=0; x<MAZE_SIZE; x++){
						if(v==Vector(x, y)) os << " " << C_YELLOW << ">^<v "[d] << C_RESET << " ";
						else os << C_CYAN << std::setw(3) << std::min(getStep(x, y), (step_t)999) << C_RESET;
						os << (maze.isKnown(x,y,Dir::East)?(maze.isWall(x,y,Dir::East)?"|":" "):(C_RED "." C_RESET));
					}
					os << std::endl;
				}
				for(uint8_t x=0; x<MAZE_SIZE; x++) os << "+" << (maze.isKnown(x,y,Dir::South)?(maze.isWall(x,y,Dir::South)?"---":"   "):(C_RED " . " C_RESET));
				os << "+" << std::endl;
			}
		}
		/** @function update
		*  @brief ステップマップの更新
		*  @param dest ステップを0とする区画の配列
		*  @param onlyCanGo true:未知の壁は通過不可能とする，false:未知の壁はないものとする
		*  @param diagonal true: 斜め直線あり false: 斜めはジグザグ
		*/
		void update(const Maze& maze, const Vectors& dest, const bool onlyCanGo = false, const bool diagonal = true)
		{
			// 全区画のステップを最大値に設定
			reset();
			// となりの区画のステップが更新されたので更新が必要かもしれない区画のキュー
			std::queue<Vector> q;
			// destに含まれる区画のステップを0とする
			for(const auto& v: dest) {
				setStep(v, 0);
				q.push(v);
			}
			// ステップの更新がなくなるまで更新処理
			while(!q.empty()){
				// 注目する区画を取得
				const Vector focus = q.front(); q.pop();
				const step_t& focus_step = getStep(focus);
				// 4方向更新がないか調べる
				for(const auto& d: Dir::All()){
					if(maze.isWall(focus, d)) continue; //< 壁があったら更新はしない
					if(onlyCanGo && !maze.isKnown(focus, d)) continue; //< onlyCanGoで未知壁なら更新はしない
					// 直線で行けるところまで更新する
					Vector next = focus;
					for(int i=0; i<MAZE_SIZE; i++){
						if(maze.isWall(next, d)) break; //< 壁があったら更新はしない
						if(onlyCanGo && !maze.isKnown(next, d)) break; //< onlyCanGoで未知壁なら更新はしない
						// となりの区画のステップが注目する区画のステップよりも大きければ更新
						next = next.next(d); //< となりの区画のステップを取得
						step_t step = focus_step + straightStepTable[i];
						if(getStep(next) < step) break; //< これより先，更新されることはない
						setStep(next, step);
						q.push(next); //< 再帰的に更新され得るのでキューにプッシュ
					}
					if(!diagonal) continue; //< 斜めなしの場合
					// 斜め直線で行けるところまで更新する
					next = focus.next(d);
					for(int i=1; i<MAZE_SIZE*2; i++){
						const Dir next_d = d+(i&1);
						if(maze.isWall(next, next_d)) break; //< 壁があったら更新はしない
						if(onlyCanGo && !maze.isKnown(next, next_d)) break; //< onlyCanGoで未知壁なら更新はしない
						// となりの区画のステップが注目する区画のステップよりも大きければ更新
						next = next.next(next_d); //< となりの区画のステップを取得
						step_t step = focus_step + straightStepTable[i]+1;
						if(getStep(next) < step) break; //< これより先，更新されることはない
						setStep(next, step);
						q.push(next); //< 再帰的に更新され得るのでキューにプッシュ
					}
					// 斜め直線で行けるところまで更新する
					next = focus.next(d);
					for(int i=1; i<MAZE_SIZE*2; i++){
						const Dir next_d = d-(i&1);
						if(maze.isWall(next, next_d)) break; //< 壁があったら更新はしない
						if(onlyCanGo && !maze.isKnown(next, next_d)) break; //< onlyCanGoで未知壁なら更新はしない
						// となりの区画のステップが注目する区画のステップよりも大きければ更新
						next = next.next(next_d); //< となりの区画のステップを取得
						step_t step = focus_step + straightStepTable[i]+1;
						if(getStep(next) < step) break; //< これより先，更新されることはない
						setStep(next, step);
						q.push(next); //< 再帰的に更新され得るのでキューにプッシュ
					}
				}
			}
		}
		/** @function updateSimple
		*   @brief シンプルに1マス1ステップのステップマップ更新
		*   @param dest ステップを0とする区画の配列(destination)
		*   @param onlyCanGo true:未知の壁は通過不可能とする，false:未知の壁はないものとする
		*/
		void updateSimple(const Maze& maze, const Vectors& dest, const bool onlyCanGo = false)
		{
			// return update(maze, dest, onlyCanGo, false);
			// 全区画のステップを最大値に設定
			reset();
			// となりの区画のステップが更新されたので更新が必要かもしれない区画のキュー
			std::queue<Vector> q;
			// destに含まれる区画のステップを0とする
			for(const auto& v: dest) {
				setStep(v, 0);
				q.push(v);
			}
			// ステップの更新がなくなるまで更新処理
			while(!q.empty()){
				// 注目する区画を取得
				const Vector focus = q.front(); q.pop();
				const step_t& focus_step = getStep(focus);
				// 4方向更新がないか調べる
				for(const auto& d: Dir::All()){
					if(maze.isWall(focus, d)) continue; //< 壁があったら更新はしない
					if(onlyCanGo && !maze.isKnown(focus, d)) continue; //< onlyCanGoで未知壁なら更新はしない
					Vector next = focus.next(d);
					if(getStep(next) <= focus_step+1) continue; //< 更新の必要がない
					setStep(next, focus_step+1);
					q.push(next); //< 再帰的に更新され得るのでキューにプッシュ
				}
			}
		}
		const Vector calcNextDirs(Maze& maze, const Vectors& dest, const Vector vec, const Dir dir, Dirs& nextDirsKnown, Dirs& nextDirCandidates)
		{
			updateSimple(maze, dest, false);
			return calcNextDirs(maze, vec, dir, nextDirsKnown, nextDirCandidates);

			updateSimple(maze, dest, false);
			const auto v = calcNextDirs(maze, vec, dir, nextDirsKnown, nextDirCandidates);
			Dirs ndcs;
			WallLogs cache;
			while(1){
				if(nextDirCandidates.empty()) break;
				const Dir d = nextDirCandidates[0]; //< 行きたい方向
				ndcs.push_back(d); //< 候補に入れる
				if(maze.isKnown(v, d)) break; //< 既知なら終わり
				cache.push_back(WallLog(v, d, false)); //< 壁をたてるのでキャッシュしておく
				maze.setWall (v, d, true); //< 壁をたてる
				maze.setKnown (v, d, true); //< 既知とする
				Dirs tmp_nds;
				// 行く方向を計算しなおす
				updateSimple(maze, dest, false);
				calcNextDirs(maze, v, d, tmp_nds, nextDirCandidates);
				if(!tmp_nds.empty()) nextDirCandidates = tmp_nds; //< 既知区間になった場合
			}
			// キャッシュを復活
			for(auto wl: cache) {
				maze.setWall (Vector(wl), wl.d, false);
				maze.setKnown(Vector(wl), wl.d, false);
			}
			nextDirCandidates = ndcs;
			return v;
		}

	private:
		step_t stepMap[MAZE_SIZE][MAZE_SIZE]; /**< @brief ステップ数 */
		step_t straightStepTable[MAZE_SIZE*2]; //**< @brief 最短経路導出用の加速を考慮したステップリスト */

		/** @function calcStraightStepTable
		*   @brief 最短経路導出用の加速を考慮したステップリストを算出する関数
		*   高速化のため，あらかじめ計算を終えておく．
		*/
		void calcStraightStepTable()
		{
			const float a = 9000;
			const float v0 = 300;
			const float factor = 1.0f / (sqrt(pow(v0/a,2) + 90*(MAZE_SIZE*2)/a) - sqrt(pow(v0/a,2) + 90*(MAZE_SIZE*2-1)/a));
			for(int i=0; i<MAZE_SIZE*2; i++){
				float x = 90*(i+1);
				straightStepTable[i] = (sqrt(pow(v0/a,2) + x/a) - v0/a) * factor;
			}
			for(int i=0; i<MAZE_SIZE*2; i++){
				float x = 90*(i+1);
				straightStepTable[i] = (sqrt(pow(v0/a,2) + x/a) - v0/a) * factor;
			}
		}
		/** @function calcNextDirs
		*   @brief ステップマップにより次に行くべき方向列を生成する
		*   @return true:成功, false:失敗(迷子)
		*/
		Vector calcNextDirs(const Maze& maze, const Vector& start_v, const Dir& start_d, Dirs& nextDirsKnown, Dirs& nextDirCandidates) const
		{
			// ステップマップから既知区間進行方向列を生成
			nextDirsKnown.clear();
			auto focus_v = start_v;
			auto focus_d = start_d;
			while(1){
				if(maze.unknownCount(focus_v)) break; //< 未知壁があれば，既知区間は終了
				step_t min_step = MAZE_STEP_MAX;
				// 周囲の区画のうち，最小ステップの方向を求める
				for(const auto d: {focus_d+0, focus_d+1, focus_d-1, focus_d+2}){
					if(maze.isWall(focus_v, d)) continue; //< 壁があったら行けない
					step_t next_step = getStep(focus_v.next(d));
					if(min_step > next_step) {
						min_step = next_step;
						focus_d = d;
					}
				}
				if(getStep(focus_v) <= min_step) break; //< 永遠ループ防止
				nextDirsKnown.push_back(focus_d); //< 既知区間移動
				focus_v = focus_v.next(focus_d); //< 位置を更新
			}
			//< 既知区間終わり
			// ステップマップから未知壁方向の優先順位付方向列を生成
			Dirs dirs;
			// 方向の候補を抽出
			for(const auto d: {focus_d+0, focus_d+1, focus_d-1, focus_d+2}) if(!maze.isWall(focus_v, d) && getStep(focus_v.next(d))!=MAZE_STEP_MAX) dirs.push_back(d);
			// ステップが小さい順に並べ替え
			std::sort(dirs.begin(), dirs.end(), [&](const Dir& d1, const Dir& d2){
				return getStep(focus_v.next(d1)) < getStep(focus_v.next(d2)); //< 低コスト優先
			});
			// 未知壁優先で並べ替え
			std::sort(dirs.begin(), dirs.end(), [&](const Dir& d1, const Dir& d2){
				return !maze.unknownCount(focus_v.next(d2)); //< 未知壁優先
			});
			// 結果を代入
			nextDirCandidates = dirs;
			return focus_v;
		}
	};
}
