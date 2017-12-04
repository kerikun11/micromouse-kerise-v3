#pragma once

#include <Arduino.h>

#include "buzzer.h"
#include "button.h"
#include "led.h"
#include "motor.h"
#include "imu.h"
#include "encoder.h"
#include "reflector.h"
#include "tof.h"

class UserInterface {
  public:
    UserInterface() {}
    static int waitForSelect(int range = 16) {
      uint8_t prev = 0;
      while (1) {
        delay(10);
        uint8_t value = (enc.position(0) + enc.position(1)) / 10;
        value %= range;
        if (value != prev) {
          prev = value;
          for (int i = 0; i < value / 16; i++) bz.play(Buzzer::SELECT);
          led = value;
        }
        if (btn.pressed) {
          btn.flags = 0;
          bz.play(Buzzer::CONFIRM);
          log_i("waitForSelect() => %d", value);
          return value;
        }
        if (btn.long_pressed_1) {
          btn.flags = 0;
          bz.play(Buzzer::CANCEL);
          log_i("waitForSelect() => -1");
          return -1;
        }
      }
      return -1;
    }
    static bool waitForCover(bool side = false) {
      while (1) {
        delay(1);
        if (!side && ref.front(0) > 200 && ref.front(1) > 200) {
          bz.play(Buzzer::CONFIRM);
          log_i("waitForCover(front) => true");
          return true;
        }
        if (side && ref.side(0) > 500 && ref.side(1) > 500) {
          bz.play(Buzzer::CONFIRM);
          log_i("waitForCover(side) => true");
          return true;
        }
        if (btn.pressed) {
          btn.flags = 0;
          bz.play(Buzzer::CANCEL);
          log_i("waitForCover() => true");
          return false;
        }
      }
    }
    static bool waitForFix() {
      int fix_count = 0;
      while (1) {
        delay(1);
        if (fabs(imu.gyro.x) < 0.01f * PI && fabs(imu.gyro.y) < 0.01f * PI && fabs(imu.gyro.z) < 0.01f * PI) {
          if (fix_count++ > 1000) {
            bz.play(Buzzer::CONFIRM);
            log_i("waitForFix() => true");
            return true;
          }
        } else {
          fix_count = 0;
        }
        if (btn.pressed) {
          btn.flags = 0;
          bz.play(Buzzer::CANCEL);
          log_i("waitForFix() => false");
          return false;
        }
      }
    }
    static void batteryLedIndicate(const float voltage) {
      if (voltage < 3.9f) led = 0x01;
      else if (voltage < 4.1f) led = 0x03;
      else if (voltage < 4.3f) led = 0x07;
      else led = 0x0F;
    }
    static void batteryCheck() {
      float voltage = 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
      batteryLedIndicate(voltage);
      printf("Battery Voltage: %.3f\n", voltage);
      if (voltage < 3.8f) {
        printf("Battery Low!\n");
        bz.play(Buzzer::LOW_BATTERY);
        while (!btn.pressed)delay(100);
        btn.flags = 0;
        //    led = 0;
        //    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
        //    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
        //    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
        //    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
        //    esp_deep_sleep_start();
      }
    }
  private:

};

