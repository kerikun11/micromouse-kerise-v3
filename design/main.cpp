#include <cstdio>
#include <fstream>
#include <iostream>
#include "AccelDesigner.h"

int main(void){
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  std::ofstream f("out.csv");
  AccelDesigner sd;
  sd.reset(9000, sd.v_end(), 1200, 1200,   90, sd.x_end()); sd.printCsv(f); std::cout << "main x_end: " << sd.x_end() << std::endl;
  sd.reset(9000, sd.v_end(), 1200,  900,  720, sd.x_end()); sd.printCsv(f); std::cout << "main x_end: " << sd.x_end() << std::endl;
  sd.reset(9000, sd.v_end(), 1200,  300,  360, sd.x_end()); sd.printCsv(f); std::cout << "main x_end: " << sd.x_end() << std::endl;
  sd.reset(9000, sd.v_end(), 3600, 1200, 2880, sd.x_end()); sd.printCsv(f); std::cout << "main x_end: " << sd.x_end() << std::endl;
  sd.reset(9000, sd.v_end(), 1200,    0,   90, sd.x_end()); sd.printCsv(f); std::cout << "main x_end: " << sd.x_end() << std::endl;

  // sd.reset(3000, sd.v_end(), 2400, 1200, sd.x_end(),  sd.x_end() +2880); sd.printCsv(f);
  return 0;
}
