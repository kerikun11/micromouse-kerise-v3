#include <cstdio>
#include <fstream>
#include <iostream>
#include "AccelDesigner.h"

int main(void){
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  std::ofstream f("out.csv");
  AccelDesigner sd;
  sd.reset(9000, sd.v_end(), 1200, 900, sd.x_end(),  sd.x_end() +  90); sd.printCsv(f);
  sd.reset(9000, sd.v_end(), 1200, 900, sd.x_end(),  sd.x_end() + 720); sd.printCsv(f);
  sd.reset(9000, sd.v_end(), 1200, 300, sd.x_end(),  sd.x_end() + 360); sd.printCsv(f);
  sd.reset(3000, sd.v_end(), 2400, 1200, sd.x_end(),  sd.x_end() + 2880); sd.printCsv(f);
  sd.reset(9000, sd.v_end(), 1200, 0, sd.x_end(),  sd.x_end() +  90); sd.printCsv(f);
  return 0;
}
