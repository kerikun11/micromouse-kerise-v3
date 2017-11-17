/**
  KERISE v3-2
  Author:  kerikun11 (Github: kerikun11)
  Date:    2017.10.25
*/

#include <WiFi.h>
#include <SPIFFS.h>
#include <Preferences.h>
Preferences pref;

#include "config.h"

/* Hardware */
#include "UserInterface.h"
#include "motor.h"
#include "axis.h"
#include "encoder.h"
#include "reflector.h"
#include "tof.h"
Buzzer bz(BUZZER_PIN, LEDC_CH_BUZZER);
Button btn(BUTTON_PIN);
LED led(LED_PINS);
Motor mt;
Fan fan;
Axis axis;
Encoder enc;
Reflector ref(PR_TX_PINS, PR_RX_PINS);
ToF tof(TOF_SDA_PIN, TOF_SCL_PIN);

/* Software */
#include "Emergency.h"
#include "debug.h"
#include "logger.h"
//#include "BLETransmitter.h"
#include "WallDetector.h"
#include "SpeedController.h"
#include "SearchRun.h"
#include "FastRun.h"
#include "MazeSolver.h"
Emergency em;
ExternalController ec;
Logger lg;
//BLETransmitter ble;
WallDetector wd;
SpeedController sc;
SearchRun sr;
FastRun fr;
MazeSolver ms;

#include "utils.h"

//#define printf lg.printf

void task(void* arg) {
  portTickType xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    //    ref.csv(); vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    //    tof.csv(); vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    //    ref.print(); vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
    //    wd.print(); vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
    //    printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", sc.target.trans, sc.actual.trans, sc.enconly.trans, sc.Kp * sc.proportional.trans, sc.Ki * sc.integral.trans, sc.Kd * sc.differential.trans, sc.Kp * sc.proportional.trans + sc.Ki * sc.integral.trans + sc.Kd * sc.differential.trans);
    //    printf("0,%f,%f,%f\n", PI, -PI, axis.gyro.z * 10);
  }
}

void setup() {
  WiFi.mode(WIFI_OFF);
  //  Serial.begin(115200);
  Serial.begin(2000000);
  pinMode(RX, INPUT_PULLUP);
  printf("\n************ KERISE v3-2 ************\n");
  led = 0xf;
  batteryCheck();
  bz.play(Buzzer::BOOT);
  log_i("tskNO_AFFINITY => %d", tskNO_AFFINITY);

  pref.begin("mouse", false);
  restore();

  if (!SPIFFS.begin(true)) log_e("SPIFFS Mount Failed");
  axis.begin(true);
  enc.begin(false);
  ref.begin();
  if (!tof.begin()) bz.play(Buzzer::ERROR);
  wd.begin();
  em.init();
  ec.init();
  //  ble.begin();

  xTaskCreate(task, "test", 4096, NULL, 0, NULL);
}

const int searchig_time_ms = 5 * 60 * 1000;
bool timeup = false;

void loop() {
  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    task();
  }
  if (btn.long_pressing_1) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    ms.print();
    lg.print();
  }
  if (!timeup && millis() > searchig_time_ms) {
    timeup = true;
    bz.play(Buzzer::LOW_BATTERY);
    ms.forceBackToStart();
    //    ms.terminate();
  }
  delay(10);
}

void task() {
  normal_drive();
  //  position_test();
  //  trapizoid_test();
  //  straight_test();
  //  turn_test();
}

void position_test() {
  if (!waitForCover()) return;
  delay(1000);
  bz.play(Buzzer::SELECT);
  axis.calibration();
  bz.play(Buzzer::CANCEL);
  sc.enable();
  sc.set_target(0, 0);
}

void trapizoid_test() {
  if (!waitForCover()) return;
  delay(1000);
  axis.calibration();
  fan.drive(0.3);
  delay(500);
  lg.start();
  sc.enable();
  const float accel = 9000;
  const float decel = 6000;
  const float v_max = 1500;
  const float v_start = 0;
  float T = 1.5f * (v_max - v_start) / accel;
  for (int ms = 0; ms / 1000.0f < T; ms++) {
    float velocity_a = v_start + (v_max - v_start) * 6.0f * (-1.0f / 3 * pow(ms / 1000.0f / T, 3) + 1.0f / 2 * pow(ms / 1000.0f / T, 2));
    sc.set_target(velocity_a, 0);
    delay(1);
  }
  bz.play(Buzzer::SELECT);
  delay(150);
  bz.play(Buzzer::SELECT);
  for (float v = v_max; v > 0; v -= decel / 1000) {
    sc.set_target(v, 0);
    delay(1);
  }
  sc.set_target(0, 0);
  delay(150);
  bz.play(Buzzer::CANCEL);
  sc.disable();
  fan.drive(0);
  lg.end();
}

void straight_test() {
  if (!waitForCover()) return;
  delay(1000);
  bz.play(Buzzer::SELECT);
  axis.calibration();
  sc.enable();
  fan.drive(0.3);
  delay(500);
  straight_x(8 * 90 - 6 - MACHINE_TAIL_LENGTH, 1200, 0);
  sc.set_target(0, 0);
  fan.drive(0);
  delay(100);
  sc.disable();
}

void turn_test() {
  if (!waitForCover()) return;
  delay(1000);
  axis.calibration();
  bz.play(Buzzer::CONFIRM);
  lg.start();
  sc.enable();
  turn(-10 * 2 * PI);
  turn(10 * 2 * PI);
  sc.disable();
  bz.play(Buzzer::CANCEL);
  lg.end();
}

void normal_drive() {
  if (ms.isRunning()) ms.terminate();
  int mode = waitForSelect(9);
  switch (mode) {
    //* 走行
    case 0:
      if (!waitForCover()) {
        bz.play(Buzzer::SUCCESSFUL);
        if (!waitForCover()) return;
        led = 9;
        ms.start(true);
      } else {
        led = 9;
        ms.start();
      }
      break;
    //* 走行パラメータの選択 & 走行
    case 1: {
        int preset = waitForSelect(16);
        if (preset < 0) break;
        switch (preset) {
          case 0:  fr.runParameter = FastRun::RunParameter(0.7,  600, 2400, 1200); break;
          case 1:  fr.runParameter = FastRun::RunParameter(0.7,  900, 3600, 1800); break;
          case 2:  fr.runParameter = FastRun::RunParameter(0.7, 1200, 4800, 2400); break;
          case 3:  fr.runParameter = FastRun::RunParameter(0.7, 1500, 6000, 3000); break;

          case 4:  fr.runParameter = FastRun::RunParameter(0.8,  600, 2400, 1200); break;
          case 5:  fr.runParameter = FastRun::RunParameter(0.8,  900, 3600, 1800); break;
          case 6:  fr.runParameter = FastRun::RunParameter(0.8, 1200, 4800, 2400); break;
          case 7:  fr.runParameter = FastRun::RunParameter(0.8, 1500, 6000, 3000); break;

          case 8:  fr.runParameter = FastRun::RunParameter(0.9,  600, 2400, 1200); break;
          case 9:  fr.runParameter = FastRun::RunParameter(0.9,  900, 3600, 1800); break;
          case 10: fr.runParameter = FastRun::RunParameter(0.9, 1200, 4800, 2400); break;
          case 11: fr.runParameter = FastRun::RunParameter(0.9, 1500, 6000, 3000); break;

          case 12: fr.runParameter = FastRun::RunParameter(1.0,  600, 2400, 1200); break;
          case 13: fr.runParameter = FastRun::RunParameter(1.0,  900, 3600, 1800); break;
          case 14: fr.runParameter = FastRun::RunParameter(1.0, 1200, 4800, 2400); break;
          case 15: fr.runParameter = FastRun::RunParameter(1.0, 1500, 6000, 3000); break;
        }
      }
      if (!waitForCover()) return;
      led = 9;
      ms.start();
      break;
    //* 速度の設定
    case 2: {
        bz.play(Buzzer::SELECT);
        int value;
        value = waitForSelect(16);
        if (value < 0) return;
        const float curve_gain = 0.1f * value;
        value = waitForSelect(7);
        if (value < 0) return;
        const float v_max = 300.0f * value;
        value = waitForSelect(16);
        if (value < 0) return;
        const float accel = 600.0f * value;
        fr.runParameter = FastRun::RunParameter(curve_gain,  v_max, accel, accel / 2);
        bz.play(Buzzer::SUCCESSFUL);
        if (!waitForCover()) return;
        led = 9;
        ms.start();
      }
      break;
    //* 壁制御の設定
    case 3: {
        int value = waitForSelect(16);
        if (value < 0) return;
        if (!waitForCover()) return;
        fr.wallAvoidFlag = value & 0x01;
        fr.wallAvoid45Flag = value & 0x02;
        fr.wallCutFlag = value & 0x04;
        fr.V90Enabled = value & 0x08;
        bz.play(Buzzer::SUCCESSFUL);
      }
      break;
    //* ゴール区画の設定
    case 4: {
        for (int i = 0; i < 2; i++) bz.play(Buzzer::SHORT);
        int value = waitForSelect(6);
        if (value < 0) return;
        if (!waitForCover()) return;
        switch (value) {
          case 0: ms.set_goal({Vector(14, 14)}); break;
          case 1: ms.set_goal({Vector(1, 0)}); break;
          case 2: ms.set_goal({Vector(4, 4), Vector(4, 5), Vector(5, 4), Vector(5, 5)}); break;
          case 3: ms.set_goal({Vector(19, 20), Vector(19, 21), Vector(19, 22), Vector(20, 20), Vector(20, 21), Vector(20, 22), Vector(21, 20), Vector(21, 21), Vector(21, 22)}); break;
          case 4: ms.set_goal({Vector(4, 8)}); break;
        }
        bz.play(Buzzer::SUCCESSFUL);
      }
      break;
    //* 前壁キャリブレーション
    case 5:
      if (!waitForCover(true)) return;
      delay(1000);
      bz.play(Buzzer::CONFIRM);
      wd.calibrationFront();
      bz.play(Buzzer::CANCEL);
      break;
    //* 前壁補正データの保存
    case 6:
      if (!waitForCover()) return;
      if (backup()) {
        bz.play(Buzzer::SUCCESSFUL);
      } else {
        bz.play(Buzzer::ERROR);
      }
      break;
    //* 迷路データの復元
    case 7:
      bz.play(Buzzer::MAZE_RESTORE);
      if (!waitForCover()) return;
      if (ms.restore()) {
        bz.play(Buzzer::SUCCESSFUL);
      } else {
        bz.play(Buzzer::ERROR);
      }
      break;
    //* マス直線
    case 8:
      if (!waitForCover(true)) return;
      delay(1000);
      bz.play(Buzzer::CONFIRM);
      axis.calibration();
      bz.play(Buzzer::CANCEL);
      sc.enable();
      straight_x(16 * 90 - 6 - MACHINE_TAIL_LENGTH, 300, 0);
      sc.disable();
      break;
  }
}

