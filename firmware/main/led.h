#pragma once

#include <Arduino.h>
#include <vector>

class LED {
  public:
    LED(const std::vector<int> pins): pins(pins), value(0) {}
    bool begin() {
      for (auto pin : pins) pinMode(pin, OUTPUT);
      return true;
    }
    operator uint8_t() const {
      return value;
    }
    uint8_t operator=(uint8_t new_value) {
      value = new_value;
      for (int i = 0; i < pins.size(); i++) digitalWrite(pins[i], (value & (1 << i)) ? HIGH : LOW);
      return value;
    }
  private:
    const std::vector<int> pins;
    uint8_t value;
};

