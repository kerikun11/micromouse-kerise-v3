#include "Arduino.h"

extern "C" void app_main() {
  initArduino();
  setup();
  for (;;) {
    loop();
  }
}
