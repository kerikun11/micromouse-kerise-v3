#include <cstdio>
#include <fstream>
#include <iostream>
#include "AccelDesigner.h"

int main(void){
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  std::ofstream of("out.csv");
  AccelDesigner sd;
  sd.reset(9000,         -1, 2400,    0, 90*4,          1); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  sd.reset(9000, sd.v_end(),    0,    0,    0, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(9000, sd.v_end(), 1200,  900,   90, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(9000, sd.v_end(), 1200,  300,  360, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(9000, sd.v_end(), 1200, 1200,   90, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(9000, sd.v_end(), 1200,  300,  360, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(9000, sd.v_end(), 3600, 1200, 2880, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(9000, sd.v_end(), 1200,    0,   90, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(9000, sd.v_end(), 1200,    0,   90, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(9000, sd.v_end(), 1200,    0,   90, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(9000, sd.v_end(), 1200,    0,   90, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(9000, sd.v_end(), 1200,    0,   90, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  //
  // sd.reset(9000, sd.v_end(), 3600, 1200, 2880, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(9000, sd.v_end(), 1200,    0,   90, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;

  // sd.reset(6000, sd.v_end(), 2400, 1200, 1800, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(6000, sd.v_end(), 2400,  600,  360, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(6000, sd.v_end(), 2400,    0,  720, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;
  // sd.reset(18000, 0, 2700,    0,  1800, sd.x_end()); sd.printCsv(of); std::cout << "main x_end: " << sd.x_end() << std::endl;

  // for(float t=0; t<sd.t_end(); t+=0.001f){
  //   printf("%f, %f, %f\n", sd.a(t), sd.v(t), sd.x(t));
  // }
  return 0;
}
