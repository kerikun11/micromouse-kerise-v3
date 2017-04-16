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
#include "MoveAction.h"
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
MoveAction ma;
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
  //  lg.start();
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
  //    if (suction) fan.drive(0.6);
  //    delay(200);
  //    lg.start();
  //    sc.enable(suction);
  //    const float accel = 9000;
  //    const float decel = 6000;
  //    const float v_max = 1800;
  //    const float v_start = 0;
  //    float T = 1.5f * (v_max - v_start) / accel;
  //    for (int ms = 0; ms / 1000.0f < T; ms++) {
  //      float velocity_a = v_start + (v_max - v_start) * 6.0f * (-1.0f / 3 * pow(ms / 1000.0f / T, 3) + 1.0f / 2 * pow(ms / 1000.0f / T, 2));
  //      sc.set_target(velocity_a, 0);
  //      delay(1);
  //      ms++;
  //    }
  //    delay(200);
  //    for (float v = v_max; v > 0; v -= decel / 1000) {
  //      sc.set_target(v, 0);
  //      delay(1);
  //    }
  //    //    sc.set_target(1200, 0);
  //    //    delay(500);
  //    //    bz.play(Buzzer::SELECT);
  //    sc.set_target(0, 0);
  //    delay(300);
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
    task();
  }
  if (btn.long_pressed_1) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    ms.printWall();
  }
}

int waitForSelect(int range = 4) {
  int prev = 0;
  while (1) {
    delay(1);
    int value = (((int)as.position(0) + (int)as.position(1)) / 5) % range;
    if (value != prev) {
      prev = value;
      //      bz.play(Buzzer::SELECT);
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
  int preset = waitForSelect();
  if (preset < 0) return;
  delay(500);
  if (!waitForCover()) {
    return;
  }
  delay(500);
  bz.play(Buzzer::SELECT);
  switch (preset) {
    case 0:
      ms.start();
      break;
    case 1:
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_TURN_LEFT_90);
      ma.set_action(MoveAction::FAST_TURN_LEFT_90);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_TURN_LEFT_90);
      ma.set_action(MoveAction::FAST_TURN_LEFT_90);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      mpu.calibration(false);
      wd.calibration();
      mpu.calibrationWait();
      ma.enable();
      break;
    case 2:
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      ma.set_action(MoveAction::FAST_TURN_LEFT_90);
      ma.set_action(MoveAction::FAST_TURN_LEFT_90);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      mpu.calibration(false);
      wd.calibration();
      mpu.calibrationWait();
      ma.enable();
      break;
    case 3:
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_TURN_LEFT_90);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_TURN_LEFT_90);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      ma.set_action(MoveAction::FAST_TURN_RIGHT_90);
      ma.set_action(MoveAction::FAST_GO_STRAIGHT);
      mpu.calibration(false);
      wd.calibration();
      mpu.calibrationWait();
      ma.enable();
      break;
  }
}

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

