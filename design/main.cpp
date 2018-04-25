#include <cstdio>
#include "AccelDesigner.h"

int main(void){
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  AccelDesigner sd;
  // sd.reset(9000, sd.v_end(), 1200, 600, sd.x_end(),  sd.x_end() +  90); sd.printCsv();
  // sd.reset(9000, sd.v_end(), 1200, 900, sd.x_end(),  sd.x_end() + 720); sd.printCsv();
  // sd.reset(9000, sd.v_end(), 1200,   0, sd.x_end(),  sd.x_end() + 360); sd.printCsv();
  sd.reset(3000, sd.v_end(), 2400, 1200, sd.x_end(),  sd.x_end() + 2880); sd.printCsv();
  return 0;
}
