/**
* @file Maze.cpp
* @brief マイクロマウスの迷路クラスを定義
* @author KERI (Github: kerikun11)
* @url https://kerikeri.top/
* @date 2017.10.30
*/
#include "MazeLib/Maze.h"
#include <algorithm>

namespace MazeLib {
	/** @struct Dir
	*   @brief 迷路上の方向を定義
	*/
	const std::array<Dir, 4>& Dir::All()
	{
		static const std::array<Dir, 4> all = {East, North, West, South};
		return all;
	}
	std::ostream& operator<<(std::ostream& os, const Dir& d)
	{ return os << (char)(">^<v "[d]); }

	/** @struct Vector
	*   @brief 迷路上の座標を定義．左下の区画が (0,0) の (x,y) 平面
	*/
	const Vector Vector::next(const Dir &dir) const
	{
		switch(dir){
			case Dir::East: return Vector(x+1, y);
			case Dir::North: return Vector(x, y+1);
			case Dir::West: return Vector(x-1, y);
			case Dir::South: return Vector(x, y-1);
		}
		printf("Warning: invalid direction\n");
		return *this;
	}
	std::ostream& operator<<(std::ostream& os, const Vector& v)
	{ return os << "(" << std::setw(2) << (int)v.x << ", " << std::setw(2) << (int)v.y << ")"; }

	/** @class Maze
	*   @brief 迷路の壁情報を管理するクラス
	*/
	Maze::Maze(const char data[MAZE_SIZE+1][MAZE_SIZE+1], bool east_origin)
	{
		for(uint8_t y=0; y<MAZE_SIZE; y++)
		for(uint8_t x=0; x<MAZE_SIZE; x++){
			char c = data[MAZE_SIZE-y-1][x];
			uint8_t h = 0;
			if ('0' <= c && c <= '9') h = c-'0';
			else if('a'<=c && c<='f') h = c-'a'+10;
			if(east_origin){
				updateWall(Vector(x, y), Dir::East,  h&0x01, false);
				updateWall(Vector(x, y), Dir::North, h&0x02, false);
				updateWall(Vector(x, y), Dir::West,  h&0x04, false);
				updateWall(Vector(x, y), Dir::South, h&0x08, false);
			}else{
				updateWall(Vector(x, y), Dir::East,  h&0x02, false);
				updateWall(Vector(x, y), Dir::North, h&0x01, false);
				updateWall(Vector(x, y), Dir::West,  h&0x08, false);
				updateWall(Vector(x, y), Dir::South, h&0x04, false);
			}
		}
	}
	void Maze::reset(const bool setStartWall)
	{
		for(int8_t i=0; i<MAZE_SIZE-1; i++){
			wall[0][i]=0;
			wall[1][i]=0;
			known[0][i]=0;
			known[1][i]=0;
		}
		if(setStartWall){
			updateWall(Vector(0,0), Dir::East, true); //< start cell
			updateWall(Vector(0,0), Dir::North, false); //< start cell
		}
		wallLogs.clear();
	}
	bool Maze::canGo(const Vector& v, const Dir& d) const
	{
		return isKnown(v, d) && !isWall(v, d);
	}
	int8_t Maze::wallCount(const Vector& v) const
	{
		auto dirs = Dir::All();
		return std::count_if(dirs.begin(), dirs.end(), [&](const Dir& d){return isWall(v, d);});
	}
	int8_t Maze::unknownCount(const Vector& v) const
	{
		auto dirs = Dir::All();
		return std::count_if(dirs.begin(), dirs.end(), [&](const Dir& d){return !isKnown(v, d);});
	}
	bool Maze::updateWall(const Vector v, const Dir d, const bool b, const bool pushLog)
	{
		// ログに追加
		if(pushLog) wallLogs.push_back(WallLog(v, d, b));
		// 既知の壁と食い違いがあったら未知壁としてreturn
		if(isKnown(v, d) && isWall(v, d) != b){
			setWall(v, d, false);
			setKnown(v, d, false);
			return false;
		}
		// 未知壁なら壁情報を更新
		if(!isKnown(v, d)){
			setWall(v, d, b);
			setKnown(v, d, true);
		}
		return true;
	}
	bool Maze::resetLastWall(const int num)
	{
		for(int i=0;i<num;i++){
			if(wallLogs.empty()) return true;
			auto wl = wallLogs.back();
			setWall(Vector(wl), wl.d, false);
			setKnown(Vector(wl), wl.d, false);
			wallLogs.pop_back();
		}
		return true;
	}
	void Maze::print(std::ostream& os) const
	{
		for(int8_t y=MAZE_SIZE; y>=0; y--){
			if(y != MAZE_SIZE){
				os << '|';
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					auto v = Vector(x, y);
					if(v == start) os << " S ";
					else if(std::find(goals.begin(), goals.end(), v) != goals.end()) os << " G ";
					else               os << "   ";
					os << (isKnown(x,y,Dir::East)?(isWall(x,y,Dir::East)?"|":" "):".");
				}
				os << std::endl;
			}
			for(uint8_t x=0; x<MAZE_SIZE; x++) os << "+" << (isKnown(x,y,Dir::South)?(isWall(x,y,Dir::South)?"---":"   "):" . ");
			os << "+" << std::endl;
		}
	}
	bool Maze::parse(std::istream& is)
	{
		reset();
		goals.clear();
		for(int8_t y=MAZE_SIZE; y>=0; y--){
			if(y!=MAZE_SIZE){
				is.ignore(10, '|'); //< 次の|が出てくるまでスキップ
				for(uint8_t x=0; x<MAZE_SIZE; x++) {
					is.ignore(1); //< " " 空欄分をスキップ
					char c = is.get();
					if     (c=='S') start = Vector(x, y);
					else if(c=='G') goals.push_back(Vector(x, y));
					is.ignore(1); //< " " 空欄分をスキップ
					c = is.get();
					if     (c=='|') Maze::updateWall(Vector(x, y), Dir::East, true);
					else if(c==' ') Maze::updateWall(Vector(x, y), Dir::East, false);
				}
			}
			for(uint8_t x=0; x<MAZE_SIZE; x++){
				is.ignore(10, '+'); //< 次の+が出てくるまでスキップ
				std::string s;
				s += (char)is.get();
				s += (char)is.get();
				s += (char)is.get();
				if     (s=="---") Maze::updateWall(Vector(x, y), Dir::South, true);
				else if(s=="   ") Maze::updateWall(Vector(x, y), Dir::South, false);
			}
		}
		return true;
	}
	void Maze::printPath(std::ostream& os, const Vector start, const Dirs& dirs) const
	{
		int steps[MAZE_SIZE][MAZE_SIZE]={0};
		Vector v = start;
		int counter = 1;
		for(const auto d: dirs){
			v = v.next(d);
			steps[v.y][v.x] = counter++;
		}
		for(int8_t y=MAZE_SIZE; y>=0; y--){
			if(y != MAZE_SIZE){
				os << '|';
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					if(steps[y][x]!=0) os << C_YELLOW << std::setw(3) << steps[y][x] << C_RESET;
					else               os << "   ";
					os << (isKnown(x,y,Dir::East)?(isWall(x,y,Dir::East)?"|":" "):(C_RED "." C_RESET));
				}
				os << std::endl;
			}
			for(uint8_t x=0; x<MAZE_SIZE; x++) os << "+" << (isKnown(x,y,Dir::South)?(isWall(x,y,Dir::South)?"---":"   "):(C_RED " . " C_RESET));
			os << "+" << std::endl;
		}
	}
	bool Maze::isWall(const wall_size_t wall[2][MAZE_SIZE-1], const int8_t x, const int8_t y, const Dir d) const
	{
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
		return true; //< とりあえず壁ありとする
	}
	void Maze::setWall(wall_size_t wall[2][MAZE_SIZE-1], const int8_t x, const int8_t y, const Dir d, const bool b)
	{
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
}
