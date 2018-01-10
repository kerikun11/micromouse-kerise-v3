#pragma once

#include <Arduino.h>

/* Hardware */
#include "buzzer.h"
extern Buzzer bz;
#include "button.h"
extern Button btn;
#include "led.h"
extern LED led;
#include "imu.h"
extern IMU imu;
#include "encoder.h"
extern Encoder enc;
#include "reflector.h"
extern Reflector ref;
#include "tof.h"
extern ToF tof;

class UserInterface {
  private:
    const float thr_accel = 3 * 9807;
    const float thr_gyro = 3 * PI;
    const float wait_ms = 200;
  public:
    UserInterface() {}
    int waitForSelect(int range = 16) {
      uint8_t value = 0;
      led = value;
      while (1) {
        delay(1);
        if (imu.gyro.y > thr_gyro) {
          value += range - 1;
          value %= range;
          led = value;
          bz.play(Buzzer::SELECT);
          delay(wait_ms);
        }
        if (imu.gyro.y < -thr_gyro) {
          value += 1;
          value %= range;
          led = value;
          bz.play(Buzzer::SELECT);
          delay(wait_ms);
        }
        if (imu.accel.z > thr_accel) {
          bz.play(Buzzer::CONFIRM);
          log_i("waitForSelect() => %d", value);
          delay(wait_ms);
          return value;
        }
        if (abs(imu.accel.x) > thr_accel) {
          bz.play(Buzzer::CANCEL);
          log_i("waitForSelect() => -1");
          delay(wait_ms);
          return -1;
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
    bool waitForCover(bool side = false) {
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
        if (abs(imu.accel.x) > thr_accel) {
          bz.play(Buzzer::CANCEL);
          log_i("waitForCover() => false");
          delay(wait_ms);
          return false;
        }
        if (btn.long_pressed_1) {
          btn.flags = 0;
          bz.play(Buzzer::CANCEL);
          log_i("waitForCover() => false");
          return false;
        }
      }
    }
    //    static bool waitForFix() {
    //      int fix_count = 0;
    //      while (1) {
    //        delay(1);
    //        if (fabs(imu.gyro.x) < 0.01f * PI && fabs(imu.gyro.y) < 0.01f * PI && fabs(imu.gyro.z) < 0.01f * PI) {
    //          if (fix_count++ > 1000) {
    //            bz.play(Buzzer::CONFIRM);
    //            log_i("waitForFix() => true");
    //            return true;
    //          }
    //        } else {
    //          fix_count = 0;
    //        }
    //        if (btn.pressed) {
    //          btn.flags = 0;
    //          bz.play(Buzzer::CANCEL);
    //          log_i("waitForFix() => false");
    //          return false;
    //        }
    //      }
    //    }
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
        while (!btn.pressed) delay(100);
        btn.flags = 0;
        led = 0;
        //    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
        //    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
        //    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
        //    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
        //    esp_deep_sleep_start();
      }
    }
  private:

};

