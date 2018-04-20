#include <cstdio>
#include "AccelDesigner.h"

int main(void){
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  float offset = 0.0f;
  // { AccelDesigner sd(3000, 0, 1200, 600, 90); sd.printCsv(offset); offset += sd.x_end(); }
  // { AccelDesigner sd(3000, 600, 1200, 600, 90); sd.printCsv(offset); offset += sd.x_end(); }
  // { AccelDesigner sd(3000, 600, 1200, 300, 720); sd.printCsv(offset); offset += sd.x_end(); }
  // { AccelDesigner sd(3000, 300, 1200, 0, 360); sd.printCsv(offset); offset += sd.x_end(); }
  AccelDesigner sd;
  sd.reset(3000, sd.v_end(), 1200, 900, 60); sd.printCsv(offset); offset += sd.x_end();
  sd.reset(3000, sd.v_end(), 1200, 900, 180); sd.printCsv(offset); offset += sd.x_end();
  sd.reset(3000, sd.v_end(), 1200, 0, 60); sd.printCsv(offset); offset += sd.x_end();
  sd.reset(3000, sd.v_end(), 1200, 600, 90); sd.printCsv(offset); offset += sd.x_end();
}
