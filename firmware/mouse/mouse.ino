/*
  KERISE v3
  Author:  kerikun11 (Github: kerikun11)
  Date:    2017.02.24
*/

#include <WiFi.h>
#include <FS.h>
#include "esp_deep_sleep.h"
#include "config.h"

#include "as5145.h"
#include "UserInterface.h"
#include "Emergency.h"
#include "debug.h"
#include "logger.h"
#include "motor.h"
#include "mpu6500.h"
#include "reflector.h"
#include "WallDetector.h"
#include "SpeedController.h"
#include "FastRun.h"
#include "SearchRun.h"
#include "MazeSolver.h"

AS5145 as;
Buzzer bz(BUZZER_PIN, LEDC_BUZZER_CH);
Button btn(BUTTON_PIN);
LED led(LED_L_PIN, LED_R_PIN);
Emergency em;
ExternalController ec;
Logger lg;
Motor mt;
Fan fan;
MPU6500 mpu;
Reflector ref;
WallDetector wd;
SpeedController sc;
FastRun fr;
SearchRun sr;
MazeSolver ms;

void setup() {
  WiFi.mode(WIFI_OFF);
  Serial.begin(115200);
  pinMode(RX, INPUT_PULLUP);
  printf("\n************ KERISE v3 ************\n");
  printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  led = 3;

  float voltage = 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
  printf("Battery Voltage: %.3f\n", voltage);
  if (voltage < 3.8f) {
    printf("Battery Low!\n");
    bz.play(Buzzer::LOW_BATTERY);
    delay(3000);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
  }
  bz.play(Buzzer::BOOT);

  delay(500);
  mpu.init();
  as.init();
  em.init();
  ec.init();
  ref.init();

  wd.enable();
  //    lg.start();
}

void loop() {
  //  mpu.print();
  //  as.print();
  //  wd.print();
  //  delay(100);
  //  printf("%f,%f\n", as.position(0), as.position(1));
  //  printf("%d,%d\n", as.getPulses(0), as.getPulses(1));
  //  printf("%d,%d\n", as.getRaw(0), as.getRaw(1));
  //  printf("0,1800,%d,%d,%d,%d\n", ref.side(0), ref.front(0), ref.front(1), ref.side(1));
  //  delay(10);

  //  if (btn.pressed) {
  //    btn.flags = 0;
  //    bz.play(Buzzer::CONFIRM);
  //    delay(1000);
  //    mpu.calibration();
  //    bool suction = true;
  //    if (suction) fan.drive(0.3);
  //    delay(200);
  //    lg.start();
  //    sc.enable(suction);
  //    const float accel = 9000;
  //    const float decel = 9000;
  //    const float v_max = 1200;
  //    const float v_start = 0;
  //    float T = 1.5f * (v_max - v_start) / accel;
  //    for (int ms = 0; ms / 1000.0f < T; ms++) {
  //      float velocity_a = v_start + (v_max - v_start) * 6.0f * (-1.0f / 3 * pow(ms / 1000.0f / T, 3) + 1.0f / 2 * pow(ms / 1000.0f / T, 2));
  //      sc.set_target(velocity_a, 0);
  //      delay(1);
  //    }
  //    bz.play(Buzzer::SELECT);
  //    delay(200);
  //    bz.play(Buzzer::SELECT);
  //    for (float v = v_max; v > 0; v -= decel / 1000) {
  //      sc.set_target(v, 0);
  //      delay(1);
  //    }
  //    sc.set_target(0, 0);
  //    delay(400);
  //    bz.play(Buzzer::CANCEL);
  //    sc.disable();
  //    fan.drive(0);
  //    lg.end();
  //  }
  //  if (btn.long_pressing_1) {
  //    btn.flags = 0;
  //    bz.play(Buzzer::CONFIRM);
  //    lg.print();
  //  }

  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    if (ms.isRunning()) ms.terminate();
    task();
  }
  if (btn.long_pressed_1) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    ms.print();
    //    lg.print();
  }

  //  if (btn.pressed) {
  //    btn.flags = 0;
  //    bz.play(Buzzer::CANCEL);
  //    fan.drive(0);
  //  }
  //  if (btn.long_pressed_1) {
  //    btn.flags = 0;
  //    bz.play(Buzzer::CONFIRM);
  //    fan.drive(0.4);
  //  }
}

int waitForSelect(int range = 4) {
  int prev = 0;
  while (1) {
    delay(1);
    int value = (((int)as.position(0) + (int)as.position(1)) / 5) % range;
    if (value != prev) {
      prev = value;
      for (int i = 0; i < value / 4; i++) bz.play(Buzzer::SELECT);
      //      for (int i = 0; i < value; i++) bz.play(Buzzer::SELECT);
      led = value;
    }
    if (btn.pressed) {
      btn.flags = 0;
      bz.play(Buzzer::CONFIRM);
      return value;
    }
    if (btn.long_pressed_1) {
      btn.flags = 0;
      bz.play(Buzzer::CANCEL);
      return -1;
    }
  }
  return -1;
}

void task() {
  int mode = waitForSelect(4);
  switch (mode) {
    case 0:
      if (!waitForCover()) return;
      ms.start();
      break;
    case 1: {
        int speed = waitForSelect(8);
        fr.fast_speed = 200 + 200 * speed;
      }
      if (!waitForCover()) return;
      ms.start();
      break;
    case 2: {
        int gain = waitForSelect(8);
        fr.fast_curve_gain = 0.1f + 0.1f * gain;
      }
      if (!waitForCover()) return;
      ms.start();
      break;
    //    case 3: {
    //        ms.set_goal({Vector(0, 1)});
    //      }
    //      break;
    case 3:
      fr.set_path("srsssrlssllrlrrlrlsslrrllrsrllrsllrlrlslrsss");
      bz.play(Buzzer::CONFIRM);
      mpu.calibration();
      fr.enable();
      fr.waitForEnd();
      fr.disable();

      break;
  }
  bz.play(Buzzer::SELECT);
}

//void task() {
//  int preset = waitForSelect();
//  if (preset < 0) return;
//  delay(500);
//  if (!waitForCover()) {
//    return;
//  }
//  delay(1000);
//  bz.play(Buzzer::SELECT);
//  switch (preset) {
//    case 0:
//      fr.set_speed();
//      ms.start();
//      break;
//    case 1:
//      ms.start();
//      break;
//    case 2:
//      fr.set_speed();
//      ms.start();
//      break;
//    case 3:
//      fr.set_speed();
//      ms.start();
//      break;
//  }
//}

bool waitForCover() {
  while (1) {
    delay(1);
    if (ref.front(0) > 1000 && ref.front(1) > 1000) {
      bz.play(Buzzer::CONFIRM);
      return true;
    }
    if (btn.pressed) {
      btn.flags = 0;
      bz.play(Buzzer::CANCEL);
      return false;
    }
  }
}

