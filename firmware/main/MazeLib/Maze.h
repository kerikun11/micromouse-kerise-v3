/**
* @file Maze.h
* @brief マイクロマウスの迷路クラスを定義
* @author KERI (Github: kerikun11)
* @url https://kerikeri.top/
* @date 2017.10.30
*/
#pragma once

#include <cstdio>
#include <cstdint>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iomanip> //< for std::setw()

namespace MazeLib {
	/** @def MAZE_SIZE
	*   @brief 迷路の1辺の区画数
	*/
	#define MAZE_SIZE			32
	/** @typedef
	*   @brief 迷路のサイズのbit数の整数型
	*   32x32の迷路ならuint32_t，16x16ならuint16_t，8x8ならuint8_t
	*/
	typedef uint32_t wall_size_t;

	/** @def
	*   @brief 迷路のカラー表示切替
	*/
	#if 1
	#define C_RED     "\x1b[31m"
	#define C_GREEN   "\x1b[32m"
	#define C_YELLOW  "\x1b[33m"
	#define C_BLUE    "\x1b[34m"
	#define C_MAGENTA "\x1b[35m"
	#define C_CYAN    "\x1b[36m"
	#define C_RESET   "\x1b[0m"
	#else
	#define C_RED     ""
	#define C_GREEN   ""
	#define C_YELLOW  ""
	#define C_BLUE    ""
	#define C_MAGENTA ""
	#define C_CYAN    ""
	#define C_RESET   ""
	#endif
	#define ESC_UP(n) "\x1b["#n"A"

	/** @struct Dir
	*   @brief 迷路上の方向を定義
	*/
	struct Dir {
	public:
		/** @enum Dir::AbsoluteDir
		*   @brief 絶対方向の列挙型
		*/
		enum AbsoluteDir: int8_t { East, North, West, South, AbsMax };
		/** @enum Dir::RelativeDir
		*   @brief 相対方向の列挙型
		*/
		enum RelativeDir: int8_t { Front, Left, Back, Right, RelMax };
		/** @function Constructor
		*   @param d Absolute Direction
		*/
		Dir(const enum AbsoluteDir d = East) : d(d) {}
		Dir(const int8_t d) : d(AbsoluteDir(d&3)) {}
		/** @brief 整数へのキャスト
		*/
		operator int8_t() const { return d; }
		/** @brief 代入演算子のオーバーロード
		*/
		const Dir operator=(const Dir& obj) { d=obj.d; return *this; }
		/** @function All
		*   @brief 全方向の方向配列を生成する静的関数
		*/
		static const std::array<Dir, 4>& All();
		/** @function <<
		*/
		friend std::ostream& operator<<(std::ostream& os, const Dir& d);
	private:
		enum AbsoluteDir d; /**< @brief 方向の実体 */
	};
	/** @typedef Dirs
	*   @brief Dir構造体の動的配列
	*/
	typedef std::vector<Dir> Dirs;

	/** @struct Vector
	*   @brief 迷路上の座標を定義．左下の区画が (0,0) の (x,y) 平面
	*/
	union Vector {
	public:
		struct{
			int8_t x; /**< @brief 迷路の区画座標 */
			int8_t y; /**< @brief 迷路の区画座標 */
		};
		uint16_t all; //< まとめて扱うとき用 */
		Vector(int8_t x=0, int8_t y=0) : x(x), y(y) {} /**< @brief コンストラクタ */
		Vector(const Vector& obj) : all(obj.all) {} /**< @brief コンストラクタ */
		/** @brief 演算子のオーバーロード
		*/
		const Vector operator+(const Vector& obj) const { return Vector(x+obj.x, y+obj.y); }
		const Vector operator-(const Vector& obj) const { return Vector(x-obj.x, y-obj.y); }
		const Vector& operator=(const Vector& obj) { all=obj.all; return *this; }
		bool operator==(const Vector& obj) const { return all==obj.all; }
		bool operator!=(const Vector& obj) const { return all!=obj.all; }
		/** @function next
		*   @brief 自分の引数方向に隣接した区画のVectorを返す
		*   @param 隣接方向
		*   @return 隣接座標
		*/
		const Vector next(const Dir &dir) const;
		/** @function <<
		*   @brief 表示
		*/
		friend std::ostream& operator<<(std::ostream& os, const Vector& v);
	};
	/** @typedef Vectors
	*   @brief Vector構造体の動的配列
	*/
	typedef std::vector<Vector> Vectors;

	/** @union WallLog
	*   @brief 区画位置，方向，壁の有無を保持する構造体
	*/
	union WallLog {
		uint16_t all; /**< @brief 全フラグ参照用 */
		struct {
			int8_t x : 6;  /**< @brief 区画のx座標 */
			int8_t y : 6;  /**< @brief 区画のx座標 */
			uint8_t d : 3;  /**< @brief 方向 */
			uint8_t b : 1;  /**< @brief 壁の有無 */
		};
		WallLog(const Vector v , const Dir d, const bool b): x(v.x), y(v.y), d(d), b(b) {}
		WallLog(const int8_t x, const int8_t y, const Dir d, const bool b): x(x), y(y), d(d), b(b) {}
		WallLog(const uint16_t all): all(all) {}
		WallLog() {}
		operator Vector(){ return Vector(x, y); }
	};
	/** @typedef WallLogs
	*   @brief WallLog構造体の動的配列
	*/
	typedef std::vector<WallLog> WallLogs;

	/** @class Maze
	*   @brief 迷路の壁情報を管理するクラス
	*/
	class Maze {
	public:
		Maze() { reset(); }
		Maze(const Vectors& goals, const Vector start = Vector(0, 0))
		: goals(goals), start(start) { reset(); }
		/** @constructor Maze
		*   @brief ファイル名から迷路をパースするコンストラクタ
		*   @param filename ファイル名
		*/
		Maze(const char* filename)
		{ std::ifstream ifs(filename); parse(ifs); }
		/** @brief 配列から迷路を読み込むコンストラクタ
		*   @param data 各区画16進表記の文字列配列
		*  例：{"abaf", "1234", "abab", "aaff"}
		*   @param east_origin true: 東から反時計回り，false: 北から時計回り に0bitから格納されている
		*/
		Maze(const char data[MAZE_SIZE+1][MAZE_SIZE+1], bool east_origin = true);
		/** @function reset
		*   @brief 迷路の初期化．壁を削除し，スタート区画を既知に
		*/
		void reset(const bool setStartWall = true);
		/** @function isWall
		*   @brief 壁の有無を返す
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @return true: 壁あり，false: 壁なし
		*/
		bool isWall(const Vector v, const Dir d) const
		{ return isWall(wall, v.x, v.y, d); }
		bool isWall(const int8_t x, const int8_t y, const Dir d) const
		{ return isWall(wall, x, y, d); }
		/** @function setWall
		*   @brief 壁を更新をする
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @param b 壁の有無 true:壁あり，false:壁なし
		*/
		void setWall(const Vector v, const Dir d, const bool b)
		{ return setWall(wall, v.x, v.y, d, b); }
		void setWall(const int8_t x, const int8_t y, const Dir d, const bool b)
		{ return setWall(wall, x, y, d, b); }
		/** @function isKnown
		*   @brief 壁が探索済みかを返す
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @return true: 探索済み，false: 未探索
		*/
		bool isKnown(const Vector& v, const Dir& d) const
		{ return isWall(known, v.x, v.y, d); }
		bool isKnown(const int8_t& x, const int8_t& y, const Dir& d) const
		{ return isWall(known, x, y, d); }
		/** @function setWall
		*   @brief 壁の既知を更新する
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @param b 壁の未知既知 true:既知，false:未知
		*/
		void setKnown(const Vector& v, const Dir& d, const bool& b)
		{ return setWall(known, v.x, v.y, d, b); }
		void setKnown(const int8_t& x, const int8_t& y, const Dir& d, const bool& b)
		{ return setWall(known, x, y, d, b); }
		/** @function canGo
		*   @brief 通過可能かどうかを返す
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @return true:既知かつ壁なし，false:それ以外
		*/
		bool canGo(const Vector& v, const Dir& d) const;
		/** @function wallCount
		*   @brief 引数区画の壁の数を返す
		*   @param v 区画の座標
		*   @return 壁の数 0~4
		*/
		int8_t wallCount(const Vector& v) const;
		/** @function unknownCount
		*   @brief 引数区画の未知壁の数を返す
		*   @param v 区画の座標
		*   @return 既知壁の数 0~4
		*/
		int8_t unknownCount(const Vector& v) const;
		/** @function updateWall
		*   @brief 既知の壁と照らしあわせながら，壁を更新する関数
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @param b 壁の有無
		*   @return true: 正常に更新された, false: 既知の情報と不一致だった
		*/
		bool updateWall(const Vector v, const Dir d, const bool b, const bool pushLog = true);

		bool resetLastWall(const int num);
		/** @function print
		*   @brief 迷路の表示
		*   @param of output-stream
		*/
		void print(std::ostream& os = std::cout) const;
		/** @function parse
		*   @brief 迷路の文字列から壁をパースする
		*   @param is input-stream
		*/
		bool parse(std::istream& is);
		/** @function printPath
		*   @brief パス付の迷路の表示
		*   @param start パスのスタート座標
		*   @param dirs 移動方向の配列
		*   @param of output-stream
		*/
		void printPath(std::ostream& os, const Vector start, const Dirs& dirs) const;
		void printPath(const Vector start, const Dirs& dirs) const
		{ printPath(std::cout, start, dirs); }

		void setGoals(const Vectors& goals) { this->goals=goals; }
		const Vectors& getGoals() const { return goals; }
		const Vector& getStart() const { return start; }
		const WallLogs& getWallLogs() const { return wallLogs; }

	private:
		wall_size_t wall[2][MAZE_SIZE-1]; /**< 壁情報 */
		wall_size_t known[2][MAZE_SIZE-1]; /**< 既知壁情報 */
		Vectors goals;
		Vector start;
		WallLogs wallLogs;

		/** function isWall
		*   @brief 引数の壁情報配列を参照する関数
		*   @param wall 壁情報の配列ポインタ
		*   @param x,y,d 区画の座標，方向
		*/
		bool isWall(const wall_size_t wall[2][MAZE_SIZE-1], const int8_t x, const int8_t y, const Dir d) const;
		/** function isWall
		*   @brief 引数の壁情報配列を更新する関数
		*   @param wall 壁情報の配列ポインタ
		*   @param x,y,d 区画の座標，方向
		*   @param b 壁の有無
		*/
		void setWall(wall_size_t wall[2][MAZE_SIZE-1], const int8_t x, const int8_t y, const Dir d, const bool b);
	};
}
