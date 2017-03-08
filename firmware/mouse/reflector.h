#pragma once

#include <Arduino.h>
#include "task_base.h"

#define REFLECTOR_TASK_PRIORITY   1
#define REFLECTOR_TASK_STACK_SIZE 512

class Reflector: private TaskBase {
  public:
    Reflector(): TaskBase("Reflector Task", REFLECTOR_TASK_PRIORITY, REFLECTOR_TASK_STACK_SIZE) {

    }
};


