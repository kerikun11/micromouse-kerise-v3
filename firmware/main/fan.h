#pragma once

#include <Arduino.h>
#include "config.h"

#define FAN_FREQUENCY 10000
#define FAN_BIT_NUM   8

class Fan {
  public:
    Fan(int pin, uint8_t channel): pin(pin), channel(channel) {
      ledcSetup(channel, FAN_FREQUENCY, FAN_BIT_NUM);
      ledcAttachPin(pin, channel);
      ledcWrite(channel, 0);
    }
    void drive(const float duty) {
      ledcWrite(channel, duty * (pow(2, FAN_BIT_NUM) - 1));
    }
  private:
    int pin;
    uint8_t channel;
};

