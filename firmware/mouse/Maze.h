#pragma once

#include <queue>
#include <vector>
#include <array>
#include <algorithm>
#include <unistd.h>

#define MAZE_SIZE      32
#define MAZE_STEP_MAX  999

#if 0
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

#define DEEPNESS 0
#define SEARCHING_ADDITIALLY_AT_START 0

typedef uint16_t step_t;

class Dir {
  public:
    enum AbsoluteDir : int8_t { East, North, West, South, AbsMax };
    enum RelativeDir : int8_t { Forward, Left, Back, Right, RelMax };
    Dir(const enum AbsoluteDir d = East) : d(d) {}
    Dir(const int8_t d) : d(AbsoluteDir(d & 3)) {}

    operator int8_t() const {
      return d;
    }
    inline const Dir operator=(const Dir& obj) {
      this->d = obj.d;
      return *this;
    }

    inline const Dir getRelative(const enum RelativeDir& rd) const {
      return Dir(rd - d);
    }
    inline const std::array<Dir, 4> ordered() const {
      std::array<Dir, 4> order{d, d + 1, d + 3, d + 2};
      return order;
    }
    static const std::array<Dir, 4>& All() {
      static const std::array<Dir, 4> all = {East, North, West, South};
      return all;
    }
  private:
    enum AbsoluteDir d;
};

struct Vector {
  Vector(int8_t x = 0, int8_t y = 0) : x(x), y(y) {}
  Vector(const Vector& obj) : x(obj.x), y(obj.y) {}
  int8_t x, y;

  inline const Vector& operator=(const Vector& obj) {
    x = obj.x;
    y = obj.y;
    return *this;
  }
  inline const bool operator==(const Vector& obj) const {
    return x == obj.x && y == obj.y;
  }
  inline const bool operator!=(const Vector& obj) const {
    return x != obj.x || y != obj.y;
  }

  inline const Vector next(const Dir &dir) const {
    switch (dir) {
      case Dir::East: return Vector(x + 1, y);
      case Dir::North: return Vector(x, y + 1);
      case Dir::West: return Vector(x - 1, y);
      case Dir::South: return Vector(x, y - 1);
    }
    printf("Warning: invalid direction\n");
    return *this;
  }
};

class Maze {
  public:
    Maze() {
      reset();
    }
    Maze(const Maze& obj) {
      *this = obj;
    }
    Maze(const char data[MAZE_SIZE + 1][MAZE_SIZE + 1], bool east_origin = true) {
      for (uint8_t y = 0; y < MAZE_SIZE; y++)
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
          char c = data[MAZE_SIZE - y - 1][x];
          uint8_t h;
          if ('0' <= c && c <= '9') {
            h = c - '0';
          } else if ('a' <= c && c <= 'f') {
            h = c - 'a' + 10;
          }
          if (east_origin) {
            updateWall(Vector(x, y), Dir::East, h & 0x01);
            updateWall(Vector(x, y), Dir::North, h & 0x02);
            updateWall(Vector(x, y), Dir::West, h & 0x04);
            updateWall(Vector(x, y), Dir::South, h & 0x08);
          } else {
            updateWall(Vector(x, y), Dir::East, h & 0x02);
            updateWall(Vector(x, y), Dir::North, h & 0x01);
            updateWall(Vector(x, y), Dir::West, h & 0x08);
            updateWall(Vector(x, y), Dir::South, h & 0x04);
          }
        }
    }
    const Maze& operator=(const Maze& obj) {
      for (int8_t i = 0; i < MAZE_SIZE - 1; i++) {
        wall[0][i] = obj.wall[0][i];
        wall[1][i] = obj.wall[1][i];
        known[0][i] = obj.known[0][i];
        known[1][i] = obj.known[1][i];
      }
      return *this;
    }
    void reset() {
      for (int8_t i = 0; i < MAZE_SIZE - 1; i++) {
        wall[0][i] = 0;
        wall[1][i] = 0;
        known[0][i] = 0;
        known[1][i] = 0;
      }
      updateWall(Vector(0, 0), Dir::East, true); //< start cell
      updateWall(Vector(0, 0), Dir::North, false); //< start cell
    }
    bool isWall(const Vector& v, const Dir& d) const {
      return isWall(v.x, v.y, d);
    }
    bool isWall(const int8_t& x, const int8_t& y, const Dir& d) const {
      switch (d) {
        case Dir::East:
          if (x < 0 || x > MAZE_SIZE - 2) {
            return true;
          }
          if (y < 0 || y > MAZE_SIZE - 1) {
            return true;
          }
          return wall[1][x] & (1 << y);
        case Dir::North:
          if (x < 0 || x > MAZE_SIZE - 1) {
            return true;
          }
          if (y < 0 || y > MAZE_SIZE - 2) {
            return true;
          }
          return wall[0][y] & (1 << x);
        case Dir::West:
          if (x - 1 < 0 || x - 1 > MAZE_SIZE - 2) {
            return true;
          }
          if (y < 0 || y > MAZE_SIZE - 1) {
            return true;
          }
          return wall[1][x - 1] & (1 << y);
        case Dir::South:
          if (x < 0 || x > MAZE_SIZE - 1) {
            return true;
          }
          if (y - 1 < 0 || y - 1 > MAZE_SIZE - 2) {
            return true;
          }
          return wall[0][y - 1] & (1 << x);
      }
      printf("Warning: invalid direction\n");
      return true;
    }
    void setWall(const Vector& v, const Dir& d, const bool& b) {
      return setWall(v.x, v.y, d, b);
    }
    void setWall(const int8_t& x, const int8_t& y, const Dir& d, const bool& b) {
      switch (d) {
        case Dir::East:
          if (x < 0 || x > MAZE_SIZE - 2) {
            return;
          }
          if (y < 0 || y > MAZE_SIZE - 1) {
            return;
          }
          if (b) wall[1][x] |= (1 << y); else wall[1][x] &= ~(1 << y); return;
        case Dir::North:
          if (x < 0 || x > MAZE_SIZE - 1) {
            return;
          }
          if (y < 0 || y > MAZE_SIZE - 2) {
            return;
          }
          if (b) wall[0][y] |= (1 << x); else wall[0][y] &= ~(1 << x); return;
        case Dir::West:
          if (x - 1 < 0 || x - 1 > MAZE_SIZE - 2) {
            return;
          }
          if (y < 0 || y > MAZE_SIZE - 1) {
            return;
          }
          if (b) wall[1][x - 1] |= (1 << y); else wall[1][x - 1] &= ~(1 << y); return;
        case Dir::South:
          if (x < 0 || x > MAZE_SIZE - 1) {
            return;
          }
          if (y - 1 < 0 || y - 1 > MAZE_SIZE - 2) {
            return;
          }
          if (b) wall[0][y - 1] |= (1 << x); else wall[0][y - 1] &= ~(1 << x); return;
      }
    }
    bool isKnown(const Vector& v, const Dir& d) const {
      return isKnown(v.x, v.y, d);
    }
    bool isKnown(const int8_t& x, const int8_t& y, const Dir& d) const {
      switch (d) {
        case Dir::East:
          if (x < 0 || x > MAZE_SIZE - 2) {
            return true;
          }
          if (y < 0 || y > MAZE_SIZE - 1) {
            return true;
          }
          return known[1][x] & (1 << y);
        case Dir::North:
          if (x < 0 || x > MAZE_SIZE - 1) {
            return true;
          }
          if (y < 0 || y > MAZE_SIZE - 2) {
            return true;
          }
          return known[0][y] & (1 << x);
        case Dir::West:
          if (x - 1 < 0 || x - 1 > MAZE_SIZE - 2) {
            return true;
          }
          if (y < 0 || y > MAZE_SIZE - 1) {
            return true;
          }
          return known[1][x - 1] & (1 << y);
        case Dir::South:
          if (x < 0 || x > MAZE_SIZE - 1) {
            return true;
          }
          if (y - 1 < 0 || y - 1 > MAZE_SIZE - 2) {
            return true;
          }
          return known[0][y - 1] & (1 << x);
      }
      printf("Warning: invalid direction\n");
      return false;
    }
    void setKnown(const Vector& v, const Dir& d, const bool& b) {
      return setKnown(v.x, v.y, d, b);
    }
    void setKnown(const int8_t& x, const int8_t& y, const Dir& d, const bool& b) {
      switch (d) {
        case Dir::East:
          if (x < 0 || x > MAZE_SIZE - 2) {
            return;
          }
          if (y < 0 || y > MAZE_SIZE - 1) {
            return;
          }
          if (b) known[1][x] |= (1 << y); else known[1][x] &= ~(1 << y); return;
        case Dir::North:
          if (x < 0 || x > MAZE_SIZE - 1) {
            return;
          }
          if (y < 0 || y > MAZE_SIZE - 2) {
            return;
          }
          if (b) known[0][y] |= (1 << x); else known[0][y] &= ~(1 << x); return;
        case Dir::West:
          if (x - 1 < 0 || x - 1 > MAZE_SIZE - 2) {
            return;
          }
          if (y < 0 || y > MAZE_SIZE - 1) {
            return;
          }
          if (b) known[1][x - 1] |= (1 << y); else known[1][x - 1] &= ~(1 << y); return;
        case Dir::South:
          if (x < 0 || x > MAZE_SIZE - 1) {
            return;
          }
          if (y - 1 < 0 || y - 1 > MAZE_SIZE - 2) {
            return;
          }
          if (b) known[0][y - 1] |= (1 << x); else known[0][y - 1] &= ~(1 << x); return;
      }
    }
    bool canGo(const Vector& v, const Dir& d) const {
      return isKnown(v, d) && !isWall(v, d);
    }
    int8_t nWall(const Vector& v) const {
      int8_t n = 0;
      for (auto d : Dir::All()) if (isWall(v, d)) n++;
      return n;
    }
    int8_t nKnown(const Vector& v) const {
      int8_t n = 0;
      for (auto d : Dir::All()) if (isKnown(v, d)) n++;
      return n;
    }
    void updateWall(const Vector& v, const Dir& d, const bool& b) {
      setWall(v, d, b);
      setKnown(v, d, true);
    }
    void printWall(const step_t nums[MAZE_SIZE][MAZE_SIZE] = NULL, const Vector v = Vector(-1, -1)) const {
      printf("\n");
      for (int8_t y = MAZE_SIZE - 1; y >= 0; y--) {
        for (uint8_t x = 0; x < MAZE_SIZE; x++)
          printf("+%s" C_RESET, isKnown(x, y, Dir::North) ? (isWall(x, y, Dir::North) ? "---" : "   ") : C_RED " - ");
        printf("+\n");
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
          printf("%s" C_RESET, isKnown(x, y, Dir::West) ? (isWall(x, y, Dir::West) ? "|" : " ") : C_RED ":");
          if (nums != NULL) printf("%s%3d" C_RESET, v == Vector(x, y) ? C_YELLOW : C_CYAN, nums[y][x]);
          else printf("%s" C_RESET, v == Vector(x, y) ? (C_YELLOW " X ") : "   ");
        }
        printf("%s" C_RESET, isKnown(MAZE_SIZE - 1, y, Dir::East) ? (isWall(MAZE_SIZE - 1, y, Dir::East) ? "|" : " ") : C_RED ":");
        printf("\n");
      }
      for (uint8_t x = 0; x < MAZE_SIZE; x++)
        printf("+%s" C_RESET, isKnown(x, 0, Dir::South) ? (isWall(x, 0, Dir::South) ? "---" : "   ") : C_RED " - ");
      printf("+\n");
    }
    void printPath(const Vector start, const std::vector<Dir>& dirs) const {
      step_t steps[MAZE_SIZE][MAZE_SIZE] = {0};
      Vector v = start;
      int counter = 1;
      for (auto d : dirs) {
        v = v.next(d);
        steps[v.y][v.x] = counter++;
      }
      printf("\n");
      for (int8_t y = MAZE_SIZE - 1; y >= 0; y--) {
        for (uint8_t x = 0; x < MAZE_SIZE; x++)
          printf("+%s" C_RESET, isKnown(x, y, Dir::North) ? (isWall(x, y, Dir::North) ? "---" : "   ") : C_RED " - ");
        printf("+\n");
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
          printf("%s" C_RESET, isKnown(x, y, Dir::West) ? (isWall(x, y, Dir::West) ? "|" : " ") : C_RED ":");
          if (steps[y][x] != 0) printf("%s%3d" C_RESET, C_YELLOW, steps[y][x]);
          else printf("%s", "   ");
        }
        printf("%s" C_RESET, isKnown(MAZE_SIZE - 1, y, Dir::East) ? (isWall(MAZE_SIZE - 1, y, Dir::East) ? "|" : " ") : C_RED ":");
        printf("\n");
      }
      for (uint8_t x = 0; x < MAZE_SIZE; x++)
        printf("+%s" C_RESET, isKnown(x, 0, Dir::South) ? (isWall(x, 0, Dir::South) ? "---" : "   ") : C_RED " - ");
      printf("+\n");
    }
  private:
    uint32_t wall[2][MAZE_SIZE - 1];
    uint32_t known[2][MAZE_SIZE - 1];
};

class StepMap {
  public:
    StepMap(Maze& maze) : maze(maze) {
      reset();
    }
    enum Purpose : int8_t {
      Goal,
      Start,
      General,
      PurposeMax,
    };
    const StepMap& operator=(StepMap& obj) {
      for (int8_t y = 0; y < MAZE_SIZE; y++)
        for (uint8_t x = 0; x < MAZE_SIZE; x++)
          for (int sp = 0; sp < PurposeMax; ++sp)
            getStep(x, y, static_cast<Purpose>(sp)) = obj.getStep(x, y, static_cast<Purpose>(sp));
      return *this;
    }
    void reset() {
      for (int8_t y = 0; y < MAZE_SIZE; y++)
        for (uint8_t x = 0; x < MAZE_SIZE; x++)
          for (int sp = 0; sp < PurposeMax; ++sp)
            getStep(x, y, static_cast<Purpose>(sp)) = 0;
    }
    inline step_t& getStep(const Vector& v, const enum Purpose& sp = Goal) {
      return getStep(v.x, v.y, sp);
    }
    inline step_t& getStep(const int8_t& x, const int8_t& y, const enum Purpose& sp = Goal) {
      static step_t outside;
      outside = MAZE_STEP_MAX;
      if (x < 0 || y < 0 || x > MAZE_SIZE - 1 || y > MAZE_SIZE - 1) {
        printf("Warning: pefered out of field ------------------------------------------> %2d, %2d\n", x, y);
        return outside;
      }
      return stepMap[sp][y][x];
    }
    void print(const Vector& v = Vector(-1, -1), const enum Purpose& sp = Goal) const {
      maze.printWall(stepMap[sp], v);
    }
    void update(const std::vector<Vector>& dest, const enum Purpose& sp, const bool& onlyCanGo = false) {
      for (uint8_t y = 0; y < MAZE_SIZE; y++)
        for (uint8_t x = 0; x < MAZE_SIZE; x++)
          getStep(x, y, sp) = MAZE_STEP_MAX;
      std::queue<Vector> q;
      for (auto v : dest) {
        getStep(v, sp) = 0;
        q.push(v);
      }
      while (!q.empty()) {
        Vector focus = q.front(); q.pop();
        step_t focus_step = getStep(focus, sp);
        for (Dir d : Dir::All()) {
          Vector next = focus.next(d);
          if (maze.isWall(focus, d)) continue;
          if (onlyCanGo && !maze.isKnown(focus, d)) continue;
          if (getStep(next, sp) > focus_step + 1) {
            getStep(next, sp) = focus_step + 1;
            q.push(next);
          }
        }
      }
    }
  private:
    Maze& maze;
    step_t stepMap[PurposeMax][MAZE_SIZE][MAZE_SIZE];
};

class Agent {
  public:
    Agent(Maze& maze, const std::vector<Vector>& goal) : maze(maze), stepMap(maze), goal(goal) {
      reset(goal);
    }
    enum State {
      IDOLE,
      SEARCHING_FOR_GOAL,
      REACHED_GOAL,
      SEARCHING_ADDITIONALLY,
      BACKING_TO_START,
      REACHED_START,
      FORCE_BACKING_TO_START,
      GOT_LOST,
    };
    static const char* stateString(const enum State s) {
      static const char* str[] = {
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
    void reset() {
      stepMap.reset();
      curVec = Vector(0, 0);
      curDir = Dir::North;
      state = IDOLE;
      calcNextDir();
    }
    void reset(const std::vector<Vector>& goal) {
      this->goal = goal;
      reset();
    }
    void forceBackToStart() {
      if (state != REACHED_START) state = FORCE_BACKING_TO_START;
    }
    void updateCurVecDir(const Vector& v, const Dir& d) {
      curVec = v;
      curDir = d;
    }
    void updateWall(const Vector& v, const Dir& d, const bool& b) {
      maze.updateWall(v, d, b);
    }

    bool calcNextDir() {
      state = calcNextDir(curVec, curDir, state);
      return state != GOT_LOST;
    }
    bool calcShortestDirs() {
      stepMap.update(goal, StepMap::Goal, true);
      shortestDirs.clear();
      Vector v = start;
      Dir dir = Dir::North;
      Dir prev_dir = Dir::North;
      while (1) {
        std::vector<Dir> dirs;
        if (Dir(dir - prev_dir) == Dir::Left) dirs = {Dir(dir + 3), dir, Dir(dir + 1)};
        else if (Dir(dir - prev_dir) == Dir::Right) dirs = {Dir(dir + 1), dir, Dir(dir + 3)};
        else dirs = {dir, Dir(dir + 1), Dir(dir + 3)};
        auto it = std::find_if(dirs.begin(), dirs.end(), [&](const Dir & d) {
          if (!maze.canGo(v, d)) return false;
          return stepMap.getStep(v.next(d)) == stepMap.getStep(v) - 1;
        });
        if (it == dirs.end()) return false;
        prev_dir = dir;
        dir = *it;
        v = v.next(dir);
        shortestDirs.push_back(dir);
        if (stepMap.getStep(v) == 0) break;
      }
      while (maze.canGo(v, dir)) {
        shortestDirs.push_back(dir);
        v = v.next(dir);
      }
      return true;
    }
    const std::vector<std::vector<Dir>> calcNextDirInAdvance() {
      calcNextDir();
      std::vector<std::vector<Dir>> nextDirss;
      while (1) {
        auto dirs = curDir.ordered();
        auto it = std::find_if(dirs.begin(), dirs.end(), [&](const Dir & d) {
          if (maze.isWall(curVec, d)) return false;
          return stepMap.getStep(curVec.next(d)) == stepMap.getStep(curVec) - 1;
        });
        if (it == dirs.end()) break;
        printf("curVec:(%d,%d), curDir:%d, *it:%d\n", curVec.x, curVec.y, int8_t(curDir), int8_t(*it));
        if (maze.isKnown(curVec, *it)) {
          printf("isKnown; break;\n");
          calcNextDir();
          nextDirss.push_back(getNextDirs());
          break;
        }
        maze.setKnown(curVec, *it, true);
        if (calcNextDir(curVec, curDir, state) == GOT_LOST) {
          //printInfo();
          break;
        }
        maze.setKnown(curVec, *it, false);
        nextDirss.push_back(getNextDirs());
        maze.setWall(curVec, *it, true);
      }
      for (auto d : Dir::All()) if (!maze.isKnown(curVec, d)) maze.setWall(curVec, d, false);
      calcNextDir();
      for (auto nd : nextDirss) {
        printf(">");
        for (auto d : nd) printf("%d ", int8_t(d));
        printf("\n");
      }
      return nextDirss;
    }

    const State& getState() const {
      return state;
    }
    const Maze& getMaze() const {
      return maze;
    }
    const std::vector<Dir>& getNextDirs() const {
      return nextDirs;
    }
    const Vector& getCurVec() const {
      return curVec;
    }
    const Dir& getCurDir() const {
      return curDir;
    }
    const std::vector<Dir>& getShortestDirs() const {
      return shortestDirs;
    }
    void printInfo(const bool& showMaze = true) const {
      if (showMaze) {
        for (int i = 0; i < MAZE_SIZE * 2 + 4; i++) printf("\x1b[A");
        switch (state) {
          case IDOLE:
          case SEARCHING_FOR_GOAL:
            stepMap.print(curVec, StepMap::Goal);
            break;
          case REACHED_GOAL:
          case SEARCHING_ADDITIONALLY:
            stepMap.print(curVec, StepMap::General);
            break;
          case BACKING_TO_START:
            stepMap.print(curVec, StepMap::Start);
            break;
          case REACHED_START:
          case GOT_LOST:
          default:
            stepMap.print(curVec, StepMap::Goal);
            break;
        }
      }
      printf("Cur: ( %3d, %3d, %3d), State: %s       \n", curVec.x, curVec.y, uint8_t(curDir), stateString(state));
      printf("Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d\n", step, f, l, r, b);
    }
    void printPath() const {
      //for(int i=0; i<MAZE_SIZE*2+5; i++) printf("\x1b[A");
      maze.printPath(Vector(0, 0), shortestDirs);
      printf("\n\n");
      printf("Shortest Step: %d\n", shortestDirs.size() - 1);
    }
  private:
    State state;
    Maze& maze;
    StepMap stepMap;
    const Vector start{0, 0};
    std::vector<Vector> goal;
    Vector curVec;
    Dir curDir;
    std::vector<Dir> nextDirs;
    std::vector<Dir> shortestDirs;
    std::vector<Vector> candidates;
    int step = 0, f = 0, l = 0, r = 0, b = 0;

    bool calcNextDirByStepMap(const enum StepMap::Purpose& sp) {
      nextDirs.clear();
      Vector focus_v = curVec;
      Dir focus_d = curDir;
      while (1) {
        auto dirs = focus_d.ordered();
        auto it = std::find_if(dirs.begin(), dirs.end(), [&](const Dir & d) {
          if (!maze.canGo(focus_v, d)) return false;
          return stepMap.getStep(focus_v.next(d), sp) == stepMap.getStep(focus_v, sp) - 1;
        });
        if (it == dirs.end()) break;
        nextDirs.push_back(*it);
        focus_d = *it;
        focus_v = focus_v.next(*it);
      }
      if (nextDirs.empty()) {
        return false;
      }
      return true;
    }
    void findShortestCandidates() {
      stepMap.update(goal, StepMap::Goal);
      stepMap.update({start}, StepMap::Start);
      candidates.clear();
      std::vector<step_t> goal_steps;
      for (auto g : goal) goal_steps.push_back(stepMap.getStep(g, StepMap::Start));
      step_t goal_step = *(std::min_element(goal_steps.begin(), goal_steps.end()));
      for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
          Vector v(i, j);
#if DEEPNESS == 0
          if (stepMap.getStep(i, j) + stepMap.getStep(i, j, StepMap::Start) <= goal_step && maze.nKnown(Vector(i, j)) != 4) {
            candidates.push_back(v);
          }
#elif DEEPNESS == 1
          if (stepMap.getStep(i, j) != MAZE_STEP_MAX && maze.nKnown(Vector(i, j)) != 4) {
            candidates.push_back(v);
          }
#endif
        }
      }
    }
    const enum State calcNextDir(const Vector& pv, const Dir& pd, enum State state) {
      if (state == IDOLE) {
        step = 0; f = 0; l = 0; r = 0; b = 0;
        findShortestCandidates();
        if (candidates.empty()) state = BACKING_TO_START;
        else state = SEARCHING_FOR_GOAL;
#if SEARCHING_ADDITIALLY_AT_START
        state = SEARCHING_ADDITIONALLY;
#endif
      }

      if (state == SEARCHING_FOR_GOAL) {
        if (std::find(goal.begin(), goal.end(), pv) != goal.end()) {
          state = REACHED_GOAL;
          candidates = goal;
        } else {
          stepMap.update(goal, StepMap::Goal);
          if (!calcNextDirByStepMap(StepMap::Goal)) return GOT_LOST;
        }
      }

      if (state == REACHED_GOAL) {
        candidates.erase(std::find(candidates.begin(), candidates.end(), pv));
        if (candidates.empty()) {
          state = SEARCHING_ADDITIONALLY;
        } else {
          stepMap.update(candidates, StepMap::General);
          if (!calcNextDirByStepMap(StepMap::General)) return GOT_LOST;
        }
      }

      if (state == SEARCHING_ADDITIONALLY) {
        findShortestCandidates();
        if (candidates.empty()) {
          state = BACKING_TO_START;
        } else {
          stepMap.update(candidates, StepMap::General);
          if (!calcNextDirByStepMap(StepMap::General)) return GOT_LOST;
        }
      }

      if (state == BACKING_TO_START) {
        if (pv == start) {
          state = REACHED_START;
        } else {
          stepMap.update({start}, StepMap::Start);
          if (!calcNextDirByStepMap(StepMap::Start)) return GOT_LOST;
        }
      }

      if (state == FORCE_BACKING_TO_START) {
        if (pv == start) {
          state = REACHED_START;
        } else {
          stepMap.update({start}, StepMap::Start, true);
          if (!calcNextDirByStepMap(StepMap::Start)) return GOT_LOST;
        }
      }

      for (auto d : nextDirs) {
        step++;
        f += pd.getRelative(Dir::Forward) == d;
        l += pd.getRelative(Dir::Left   ) == d;
        r += pd.getRelative(Dir::Right  ) == d;
        b += pd.getRelative(Dir::Back   ) == d;
      }
      return state;
    }
};


