/**
* @file Maze.h
* @brief マイクロマウスの迷路クラスを定義
* @author KERI (Github: kerikun11)
* @date 2017.10.30
*/
#pragma once

#include <queue>
#include <vector>
#include <array>
#include <algorithm>

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
		enum RelativeDir: int8_t { Forward, Left, Back, Right, RelMax };
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
		const Dir operator=(const Dir& obj) { this->d = obj.d; return *this; }

		/** @function getRelative
		*   @brief 自オブジェクト方向から見た引数方向を返す．
		*   @param rd 対象方向
		*   @return 相対方向
		*/
		const Dir getRelative(const enum RelativeDir& rd) const { return Dir(rd-d); }
		/** @function ordered
		*   @brief 「正面，左，右，後」の順序の方向配列を生成する関数
		*/
		const std::array<Dir, 4> ordered() const {
			std::array<Dir, 4> order{d, d+1, d+3, d+2};
			return order;
		}
		/** @function All
		*   @brief 全方向の方向配列を生成する静的関数
		*/
		static const std::array<Dir, 4>& All(){
			static const std::array<Dir, 4> all = {East, North, West, South};
			return all;
		}
	private:
		enum AbsoluteDir d; /**< @brief 方向の実体 */
	};

	/** @struct Vector
	*   @brief 迷路上の座標を定義．左下の区画が (0,0) の (x,y) 平面
	*/
	union Vector {
	public:
		struct{
			int8_t x : 8; /**< @brief 迷路の区画座標 */
			int8_t y : 8; /**< @brief 迷路の区画座標 */
		};
		uint16_t all;
		Vector(int8_t x=0, int8_t y=0) : x(x), y(y) {} /**< @brief コンストラクタ */
		Vector(const Vector& obj) : x(obj.x), y(obj.y) {} /**< @brief コンストラクタ */
		/** @brief 演算子のオーバーロード
		*/
		const Vector& operator=(const Vector& obj) { all=obj.all; return *this; }
		bool operator==(const Vector& obj) const { return all==obj.all; }
		bool operator!=(const Vector& obj) const { return all!=obj.all; }
		/** @function next
		*   @brief 自分の引数方向に隣接した区画のVectorを返す
		*   @param 隣接方向
		*   @return 隣接座標
		*/
		const Vector next(const Dir &dir) const {
			switch(dir){
				case Dir::East: return Vector(x+1, y);
				case Dir::North: return Vector(x, y+1);
				case Dir::West: return Vector(x-1, y);
				case Dir::South: return Vector(x, y-1);
			}
			printf("Warning: invalid direction\n");
			return *this;
		}
	};

	/** @union WallLog
	*   @brief 区画位置，方向，壁の有無を保持する構造体
	*/
	union WallLog {
		uint16_t all; /**< @brief 全フラグ参照用 */
		struct {
			uint8_t x : 6;  /**< @brief 区画のx座標 */
			uint8_t y : 6;  /**< @brief 区画のx座標 */
			uint8_t d : 3;  /**< @brief 方向 */
			uint8_t b : 1;  /**< @brief 壁の有無 */
		};
		WallLog(const Vector& v , const Dir& d, const bool b): x(v.x), y(v.y), d(d), b(b) {}
		WallLog(const uint16_t all): all(all) {}
		WallLog() {}
	};

	/** @class Maze
	*   @brief 迷路の壁情報を管理するクラス
	*/
	class Maze {
	public:
		Maze() { reset(); } /**< @brief 空の迷路のコンストラクタ */
		Maze(const Maze& obj){ *this = obj; } /**< @brief コピーコンストラクタ */
		/** @brief 配列から迷路を読み込むコンストラクタ
		*   @param data 各区画16進表記の文字列配列
		*  例：{"abaf", "1234", "abab", "aaff"}
		*   @param east_origin true: 東から反時計回り，false: 北から時計回り に0bitから格納されている
		*/
		Maze(const char data[MAZE_SIZE+1][MAZE_SIZE+1], bool east_origin = true){
			for(uint8_t y=0; y<MAZE_SIZE; y++)
			for(uint8_t x=0; x<MAZE_SIZE; x++){
				char c = data[MAZE_SIZE-y-1][x];
				uint8_t h = 0;
				if ('0' <= c && c <= '9') h = c-'0';
				else if('a'<=c && c<='f') h = c-'a'+10;
				if(east_origin){
					updateWall(Vector(x, y), Dir::East, h&0x01);
					updateWall(Vector(x, y), Dir::North, h&0x02);
					updateWall(Vector(x, y), Dir::West, h&0x04);
					updateWall(Vector(x, y), Dir::South, h&0x08);
				}else{
					updateWall(Vector(x, y), Dir::East, h&0x02);
					updateWall(Vector(x, y), Dir::North, h&0x01);
					updateWall(Vector(x, y), Dir::West, h&0x08);
					updateWall(Vector(x, y), Dir::South, h&0x04);
				}
			}
		}
		/** @brief 代入演算子のオーバーロード，データのコピー
		*/
		const Maze& operator=(const Maze& obj){
			for(int8_t i=0; i<MAZE_SIZE-1; i++){
				wall[0][i]=obj.wall[0][i];
				wall[1][i]=obj.wall[1][i];
				known[0][i]=obj.known[0][i];
				known[1][i]=obj.known[1][i];
			}
			return *this;
		}
		/** @function reset
		*   @brief 迷路の初期化．壁を削除し，スタート区画を既知に
		*/
		void reset(){
			for(int8_t i=0; i<MAZE_SIZE-1; i++){
				wall[0][i]=0;
				wall[1][i]=0;
				known[0][i]=0;
				known[1][i]=0;
			}
			updateWall(Vector(0,0), Dir::East, true); //< start cell
			updateWall(Vector(0,0), Dir::North, false); //< start cell
		}
		/** @function isWall
		*   @brief 壁の有無を返す
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @return true: 壁あり，false: 壁なし
		*/
		bool isWall(const Vector& v, const Dir& d) const { return isWall(v.x, v.y, d); }
		bool isWall(const int8_t& x, const int8_t& y, const Dir& d) const {
			switch(d){
				case Dir::East:
				if(x<0 || x>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return true; //< 盤面外
				return wall[0][x] & (1<<y);
				case Dir::North:
				if(x<0 || x>MAZE_SIZE-1 || y<0 || y>MAZE_SIZE-2) return true; //< 盤面外
				return wall[1][y] & (1<<x);
				case Dir::West:
				if(x-1<0 || x-1>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return true; //< 盤面外
				return wall[0][x-1] & (1<<y);
				case Dir::South:
				if(x<0 || x>MAZE_SIZE-1 || y-1<0 || y-1>MAZE_SIZE-2) return true; //< 盤面外
				return wall[1][y-1] & (1<<x);
			}
			printf("Warning: invalid direction\n");
			return true;
		}
		/** @function setWall
		*   @brief 壁を更新をする
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @param b 壁の有無 true:壁あり，false:壁なし
		*/
		void setWall(const Vector& v, const Dir& d, const bool& b) { return setWall(v.x, v.y, d, b); }
		void setWall(const int8_t& x, const int8_t& y, const Dir& d, const bool& b) {
			switch(d){
				case Dir::East:
				if(x<0 || x>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return; //< 盤面外
				if(b) wall[0][x] |= (1<<y); else wall[0][x] &= ~(1<<y); return;
				case Dir::North:
				if(x<0 || x>MAZE_SIZE-1 || y<0 || y>MAZE_SIZE-2) return; //< 盤面外
				if(b) wall[1][y] |= (1<<x); else wall[1][y] &= ~(1<<x); return;
				case Dir::West:
				if(x-1<0 || x-1>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return; //< 盤面外
				if(b) wall[0][x-1] |= (1<<y); else wall[0][x-1] &= ~(1<<y); return;
				case Dir::South:
				if(x<0 || x>MAZE_SIZE-1 || y-1<0 || y-1>MAZE_SIZE-2) return; //< 盤面外
				if(b) wall[1][y-1] |= (1<<x); else wall[1][y-1] &= ~(1<<x); return;
			}
			printf("Warning: invalid direction\n");
		}
		/** @function isKnown
		*   @brief 壁が探索済みかを返す
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @return true: 探索済み，false: 未探索
		*/
		bool isKnown(const Vector& v, const Dir& d) const { return isKnown(v.x, v.y, d); }
		bool isKnown(const int8_t& x, const int8_t& y, const Dir& d) const {
			switch(d){
				case Dir::East:
				if(x<0 || x>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return true; //< 盤面外
				return known[0][x] & (1<<y);
				case Dir::North:
				if(x<0 || x>MAZE_SIZE-1 || y<0 || y>MAZE_SIZE-2) return true; //< 盤面外
				return known[1][y] & (1<<x);
				case Dir::West:
				if(x-1<0 || x-1>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return true; //< 盤面外
				return known[0][x-1] & (1<<y);
				case Dir::South:
				if(x<0 || x>MAZE_SIZE-1 || y-1<0 || y-1>MAZE_SIZE-2) return true; //< 盤面外
				return known[1][y-1] & (1<<x);
			}
			printf("Warning: invalid direction\n");
			return false;
		}
		/** @function setWall
		*   @brief 壁の既知を更新する
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @param b 壁の未知既知 true:既知，false:未知
		*/
		void setKnown(const Vector& v, const Dir& d, const bool& b) { return setKnown(v.x, v.y, d, b); }
		void setKnown(const int8_t& x, const int8_t& y, const Dir& d, const bool& b) {
			switch(d){
				case Dir::East:
				if(x<0 || x>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return; //< 盤面外
				if(b) known[0][x] |= (1<<y); else known[0][x] &= ~(1<<y); return;
				case Dir::North:
				if(x<0 || x>MAZE_SIZE-1 || y<0 || y>MAZE_SIZE-2) return; //< 盤面外
				if(b) known[1][y] |= (1<<x); else known[1][y] &= ~(1<<x); return;
				case Dir::West:
				if(x-1<0 || x-1>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return; //< 盤面外
				if(b) known[0][x-1] |= (1<<y); else known[0][x-1] &= ~(1<<y); return;
				case Dir::South:
				if(x<0 || x>MAZE_SIZE-1 || y-1<0 || y-1>MAZE_SIZE-2) return; //< 盤面外
				if(b) known[1][y-1] |= (1<<x); else known[1][y-1] &= ~(1<<x); return;
			}
			printf("Warning: invalid direction\n");
		}
		/** @function canGo
		*   @brief 通過可能かどうかを返す
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @return true:既知かつ壁なし，false:それ以外
		*/
		bool canGo(const Vector& v, const Dir& d) const {
			return isKnown(v, d) && !isWall(v, d);
		}
		/** @function wallCount
		*   @brief 引数区画の壁の数を返す
		*   @param v 区画の座標
		*   @return 壁の数 0~4
		*/
		int8_t wallCount(const Vector& v) const {
			auto dirs = Dir::All();
			return std::count_if(dirs.begin(), dirs.end(), [&](const Dir& d){return isWall(v, d);});
		}
		/** @function unknownCount
		*   @brief 引数区画の未知壁の数を返す
		*   @param v 区画の座標
		*   @return 既知壁の数 0~4
		*/
		int8_t unknownCount(const Vector& v) const {
			auto dirs = Dir::All();
			return std::count_if(dirs.begin(), dirs.end(), [&](const Dir& d){return !isKnown(v, d);});
		}
		/** @function updateWall
		*   @brief 壁を更新して既知とする関数
		*   @param v 区画の座標
		*   @param d 壁の方向
		*/
		void updateWall(const Vector& v, const Dir& d, const bool& b){
			setWall(v, d, b);
			setKnown(v, d, true);
		}
		/** @function print
		*   @brief 迷路の表示
		*   @param v ハイライト区画の座標
		*/
		void print(const Vector v = Vector(-1,-1), const Dir& d = Dir::AbsMax) const {
			printf("\n");
			for(int8_t y=MAZE_SIZE-1; y>=0; y--){
				for(uint8_t x=0; x<MAZE_SIZE; x++)
				printf("+%s" C_RESET, isKnown(x,y,Dir::North) ? (isWall(x,y,Dir::North)?"---":"   ") : C_RED " - ");
				printf("+\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s" C_RESET, isKnown(x,y,Dir::West) ? (isWall(x,y,Dir::West)?"|":" ") : C_RED ":");
					printf(" %s%c " C_RESET, C_YELLOW, v==Vector(x,y)?(">^<vX"[d]):' ');
				}
				printf("%s" C_RESET, isKnown(MAZE_SIZE-1,y,Dir::East) ? (isWall(MAZE_SIZE-1,y,Dir::East)?"|":" ") : C_RED ":");
				printf("\n");
			}
			for(uint8_t x=0; x<MAZE_SIZE; x++)
			printf("+%s" C_RESET, isKnown(x,0,Dir::South) ? (isWall(x,0,Dir::South)?"---":"   ") : C_RED " - ");
			printf("+\n");
		}
		/** @function printPath
		*   @brief パス付の迷路の表示
		*   @param start パスのスタート座標
		*   @param dirs 移動方向の配列
		*/
		void printPath(const Vector start, const std::vector<Dir>& dirs) const {
			int steps[MAZE_SIZE][MAZE_SIZE]={0};
			Vector v = start;
			int counter = 1;
			for(const auto& d: dirs){
				v = v.next(d);
				steps[v.y][v.x] = counter++;
			}
			printf("\n");
			for(int8_t y=MAZE_SIZE-1; y>=0; y--){
				for(uint8_t x=0; x<MAZE_SIZE; x++)
				printf("+%s" C_RESET, isKnown(x,y,Dir::North) ? (isWall(x,y,Dir::North)?"---":"   ") : C_RED " - ");
				printf("+\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s" C_RESET, isKnown(x,y,Dir::West) ? (isWall(x,y,Dir::West)?"|":" ") : C_RED ":");
					if(steps[y][x]!=0) printf("%s%3d" C_RESET, C_YELLOW, steps[y][x]);
					else printf("%s", "   ");
				}
				printf("%s" C_RESET, isKnown(MAZE_SIZE-1,y,Dir::East) ? (isWall(MAZE_SIZE-1,y,Dir::East)?"|":" ") : C_RED ":");
				printf("\n");
			}
			for(uint8_t x=0; x<MAZE_SIZE; x++)
			printf("+%s" C_RESET, isKnown(x,0,Dir::South) ? (isWall(x,0,Dir::South)?"---":"   ") : C_RED " - ");
			printf("+\n");
		}
		int diffKnownWall(const Maze& obj){
			int diffs = 0;
			for(int j=0; j<MAZE_SIZE-1; j++){
				for(int i=0; i<2; i++){
					wall_size_t bits = known[i][j] & (wall[i][j] ^ obj.wall[i][j]);
					diffs += popcnt(bits);
				}
			}
			return diffs;
		}
	public:
		/** @function popcnt
		*   @brief 引数のセットされているbit数を返す静的関数
		*/
		static int popcnt(uint8_t n) {
			n = (n & 0x5555555555555555) + ((n>>1)  & 0x5555555555555555);
			n = (n & 0x3333333333333333) + ((n>>2)  & 0x3333333333333333);
			n = (n & 0x0f0f0f0f0f0f0f0f) + ((n>>4)  & 0x0f0f0f0f0f0f0f0f);
			return n;
		}
		static int popcnt(uint16_t n) {
			n = (n & 0x5555555555555555) + ((n>>1)  & 0x5555555555555555);
			n = (n & 0x3333333333333333) + ((n>>2)  & 0x3333333333333333);
			n = (n & 0x0f0f0f0f0f0f0f0f) + ((n>>4)  & 0x0f0f0f0f0f0f0f0f);
			n = (n & 0x00ff00ff00ff00ff) + ((n>>8)  & 0x00ff00ff00ff00ff);
			return n;
		}
		static int popcnt(uint32_t n) {
			n = (n & 0x5555555555555555) + ((n>>1)  & 0x5555555555555555);
			n = (n & 0x3333333333333333) + ((n>>2)  & 0x3333333333333333);
			n = (n & 0x0f0f0f0f0f0f0f0f) + ((n>>4)  & 0x0f0f0f0f0f0f0f0f);
			n = (n & 0x00ff00ff00ff00ff) + ((n>>8)  & 0x00ff00ff00ff00ff);
			n = (n & 0x0000ffff0000ffff) + ((n>>16) & 0x0000ffff0000ffff);
			return n;
		}
	private:
		wall_size_t wall[2][MAZE_SIZE-1]; /**< 壁情報 */
		wall_size_t known[2][MAZE_SIZE-1]; /**< 既知壁情報 */
	};
}
