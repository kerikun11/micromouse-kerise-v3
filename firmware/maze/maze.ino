/*
  KERISE
  Author:  kerikun11 (Github: kerikun11)
*/

#include <WiFi.h>
#include <SPIFFS.h>

#include "Agent.h"

#define MAZE_GOAL {Vector(1,0)}
#define MAZE_BACKUP_SIZE 5

const char mazedata1[32 + 1][32 + 1] = {
  {"76aaaaaaaaaaaaaa2b637762376236a3"},
  {"4836a36aaaaaaaa3c355401540154961"},
  {"4b55694b6aaaaaa83555554015409695"},
  {"4b55574bca36aaaa95554015401d6969"},
  {"4a955c0b6a9caaaa3554154015d69683"},
  {"56a1568bcaaaa2aa9555c89dc969e0a1"},
  {"55695ca36aaaa96a3c9d6222b69e2829"},
  {"555616a956aaaa835623c009696a0a0b"},
  {"5555556a9c2a36a941543c9697c28283"},
  {"555555caaa8a88a35c89c36961e0a0a1"},
  {"5555556236aaaa35caa2b49601682829"},
  {"5555c9401576aa9caa29683c014a0a0b"},
  {"555563c89c1caa2ab69697c3c9c28a8b"},
  {"55c9556363563e16a96961683ea8aaa3"},
  {"556355555555ca94b697554a16aaaaa9"},
  {"555c94955c9c2a29696155c29caaaa37"},
  {"554a34bc96aa969697555ca8a36363c1"},
  {"55d69d623562bd696895436a21555435"},
  {"5568b7c01494b6969634955695555555"},
  {"55c3683c9d696969695c3555695c1555"},
  {"556883c3e296969e1e8a895543435555"},
  {"55d63cbc3c296963563623555555c955"},
  {"55e1ca369683560149540155c1543695"},
  {"55e8a3c9697c9c89d69c895569c15c35"},
  {"5563e0b69e82aaaa2963635556355695"},
  {"c15569696a3ca363d69c15c1555c1c35"},
  {"689c969e174b7c9c2963d568954b5695"},
  {"562369e294968363d69ca9c2b5ca9c35"},
  {"5400963569696954356aaaa9696aaa95"},
  {"5c8969c9ca9e8a9dc9caaaaa9696aa35"},
  {"42ab42aaaaaaaaaaaaaaaaaaa8b57749"},
  {"dcaa9caaaaaaaaaaaaaaaaaaaaa8888b"},
};

Maze maze1(mazedata1);

Maze maze;
std::deque<Maze> maze_backup;
Agent agent(maze, MAZE_GOAL);
#define MAZE_BACKUP_PATH   "/maze_backup.maze"

bool backup() {
  uint32_t us = micros();
  File file = SPIFFS.open(MAZE_BACKUP_PATH, FILE_WRITE);
  if (!file) {
    log_e("Can't open file!");
    return false;
  }
  for (auto& maze : maze_backup) {
    file.write((const uint8_t*)(&maze), sizeof(Maze));
  }
  log_d("Backup: %d [us]", micros() - us);
  return true;
}

bool restore() {
  File file = SPIFFS.open(MAZE_BACKUP_PATH, FILE_READ);
  if (!file) {
    log_e("Can't open file!");
    return false;
  }
  while (file.available() >= sizeof(Maze)) {
    uint8_t data[sizeof(Maze)];
    file.read(data, sizeof(Maze));
    Maze m;
    memcpy((uint8_t*)(&m), data, sizeof(Maze));
    maze_backup.push_back(m);
    if (maze_backup.size() > MAZE_BACKUP_SIZE) maze_backup.pop_front();
  }
  return true;
}

void setup() {
  //  WiFi.mode(WIFI_OFF);
  Serial.begin(115200);
  log_i("KERISE");
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
  }
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.println(c);
    switch (c) {
      case 'd':
        SPIFFS.remove(MAZE_BACKUP_PATH);
        break;
      case 'a':
        maze_backup.push_back(maze1);
        break;
      case 'b':
        backup();
        break;
      case 'r':
        restore();
        break;
      case 'c':
        maze_backup.resize(0);
        //        while (!maze_backup.empty()) maze_backup.pop();
        break;
      case 'p':
        for (auto& maze : maze_backup) {
          maze.print();
        }
        break;
    }
  }
}

