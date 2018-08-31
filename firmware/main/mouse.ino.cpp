/**
 * @file    mouse.ino.cpp
 * @author  KERI (Github: kerikun11)
 * @date    2017.10.25
 */

#include "global.h"
#include <SPIFFS.h>
#include <WiFi.h>
#include <cstdio>
#include <iostream>
#include <sstream>

void mainTask(void *arg);
void printTask(void *arg);
void timeKeepTask(void *arg);
std::stringstream logstream;

void setup() {
  WiFi.mode(WIFI_OFF);
  for (auto p : CONFIG_SPI_CS_PINS)
    pinMode(p, INPUT_PULLUP);
  printf("\n**************** KERISE ****************\n");
  if (!bz.begin())
    bz.play(Buzzer::ERROR);
  if (!I2C::install(I2C_PORT_NUM, I2C_SDA_PIN, I2C_SCL_PIN))
    bz.play(Buzzer::ERROR);
  if (!led.begin())
    bz.play(Buzzer::ERROR);
  ui.batteryCheck();
  bz.play(Buzzer::BOOT);

  if (!SPIFFS.begin(true))
    bz.play(Buzzer::ERROR);
  if (!SPI::busInit(CONFIG_SPI_HOST, CONFIG_SPI_SCLK_PIN, CONFIG_SPI_MISO_PIN,
                    CONFIG_SPI_MOSI_PIN, CONFIG_SPI_DMA_CHAIN))
    bz.play(Buzzer::ERROR);
  if (!imu.begin(ICM20602_SPI_HOST, ICM20602_CS_PIN))
    bz.play(Buzzer::ERROR);
  if (!enc.begin(AS5048A_SPI_HOST, AS5048A_CS_PIN))
    bz.play(Buzzer::ERROR);
  if (!ref.begin())
    bz.play(Buzzer::ERROR);
  if (!tof.begin())
    bz.play(Buzzer::ERROR);
  if (!wd.begin())
    bz.play(Buzzer::ERROR);
  em.begin();

  xTaskCreate(printTask, "print", 4096, NULL, 1, NULL);
  // xTaskCreate(timeKeepTask, "TimeKeep", 4096, NULL, 1, NULL);
  xTaskCreate(mainTask, "main", 4096, NULL, 1, NULL);
}

void loop() { delay(1000); }

void printTask(void *arg) {
  portTickType xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
    // ref.csv();
    // tof.csv();
    // imu.print();
    // wd.print();
    // printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", sc.target.trans,
    //        sc.actual.trans, sc.enconly.trans, sc.Kp * sc.proportional.trans,
    //        sc.Ki * sc.integral.trans, sc.Kd * sc.differential.trans,
    //        sc.Kp * sc.proportional.trans + sc.Ki * sc.integral.trans +
    //            sc.Kd * sc.differential.trans);
  }
}

void timeKeepTask(void *arg) {
  const int searchig_time_ms = 3 * 60 * 1000;
  while (millis() < searchig_time_ms)
    delay(1000);
  bz.play(Buzzer::LOW_BATTERY);
  ms.forceBackToStart();
  while (1)
    delay(1000);
}

#define TEST_END_REMAIN 6
#define TEST_ST_LOOK_AHEAD(v) (6 + v / 100)
#define TEST_ST_FB_GAIN 10
#define TEST_ST_TR_FB_GAIN 0

void straight_x(const float distance, const float v_max, const float v_end) {
  const float a_max = 9000;
  const float v_start = sc.actual.trans;
  AccelDesigner ad(a_max, v_start, v_max, v_end, sc.position.x,
                   distance - TEST_END_REMAIN);
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (float t = 0.0f; true; t += 0.001f) {
    Position cur = sc.position;
    //    if (cur.x > distance - TEST_END_REMAIN) break;
    if (t > ad.t_end())
      break;
    float velocity = ad.v(t) + TEST_ST_TR_FB_GAIN * (ad.x(t) - cur.x);
    float theta = atan2f(-cur.y, TEST_ST_LOOK_AHEAD(velocity)) - cur.theta;
    sc.set_target(velocity, TEST_ST_FB_GAIN * theta);
    //    wallAvoid();
    //    wallCut();
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    xLastWakeTime = xTaskGetTickCount();
  }
  sc.set_target(ad.v_end(), 0);
  //  updateOrigin(Position(distance, 0, 0));
}

void turn(const float angle) {
  const float speed = 3.6 * M_PI;
  const float accel = 36 * M_PI;
  const float decel = 36 * M_PI;
  const float back_gain = 1.0f;
  int ms = 0;
  portTickType xLastWakeTime = xTaskGetTickCount();
  while (1) {
    if (fabs(sc.actual.rot) > speed)
      break;
    float delta = sc.position.x * cos(-sc.position.theta) -
                  sc.position.y * sin(-sc.position.theta);
    if (angle > 0) {
      sc.set_target(-delta * back_gain, ms / 1000.0f * accel);
    } else {
      sc.set_target(-delta * back_gain, -ms / 1000.0f * accel);
    }
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    xLastWakeTime = xTaskGetTickCount();
    ms++;
  }
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    xLastWakeTime = xTaskGetTickCount();
    float extra = angle - sc.position.theta;
    if (fabs(sc.actual.rot) < 0.1 && fabs(extra) < 0.1)
      break;
    float target_speed = sqrt(2 * decel * fabs(extra));
    float delta = sc.position.x * cos(-sc.position.theta) -
                  sc.position.y * sin(-sc.position.theta);
    target_speed = (target_speed > speed) ? speed : target_speed;
    if (extra > 0) {
      sc.set_target(-delta * back_gain, target_speed);
    } else {
      sc.set_target(-delta * back_gain, -target_speed);
    }
  }
  sc.set_target(0, 0);
  //  updateOrigin(Position(0, 0, angle));
  //  printPosition("Turn End");
}

void position_test() {
  if (!ui.waitForCover())
    return;
  led = 9;
  delay(1000);
  sc.enable();
  sc.set_target(0, 0);
  ui.waitForCover();
  sc.disable();
}

void accel_test() {
  if (!ui.waitForCover())
    return;
  delay(500);
  logstream.flush();
  auto printLog = []() {
    logstream << "0";
    logstream << "," << sc.target.trans;
    logstream << "," << sc.actual.trans;
    logstream << "," << sc.enconly.trans;
    logstream << "," << sc.Kp * sc.proportional.trans;
    logstream << "," << sc.Ki * sc.integral.trans;
    logstream << "," << sc.Kd * sc.differential.trans;
    logstream << ","
              << sc.Kp * sc.proportional.trans + sc.Ki * sc.integral.trans +
                     sc.Kd * sc.differential.trans;
    logstream << std::endl;
  };
  imu.calibration();
  fan.drive(0.4);
  delay(500);
  sc.enable();
  const float accel = 6000;
  const float v_max = 1200;
  AccelDesigner ad(accel, 0, v_max, 0, 90 * 4);
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (float t = 0; t < ad.t_end() + 0.2f; t += 0.001f) {
    sc.set_target(ad.v(t), 0);
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    xLastWakeTime = xTaskGetTickCount();
    printLog();
  }
  sc.set_target(0, 0);
  bz.play(Buzzer::CANCEL);
  sc.disable();
  fan.drive(0);
}

void turn_test() {
  if (!ui.waitForCover())
    return;
  delay(1000);
  imu.calibration();
  bz.play(Buzzer::CONFIRM);
  sc.enable();
  turn(-10 * 2 * PI);
  turn(10 * 2 * PI);
  sc.disable();
  bz.play(Buzzer::CANCEL);
}

void trapizoid_test() {
  if (!ui.waitForCover())
    return;
  delay(1000);
  imu.calibration();
  fan.drive(0.5);
  delay(500);
  sc.enable();
  const float accel = 15000;
  const float decel = 12000;
  const float v_max = 2400;
  const float v_start = 0;
  float T = 1.5f * (v_max - v_start) / accel;
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (int ms = 0; ms / 1000.0f < T; ms++) {
    float velocity_a = v_start + (v_max - v_start) * 6.0f *
                                     (-1.0f / 3 * pow(ms / 1000.0f / T, 3) +
                                      1.0f / 2 * pow(ms / 1000.0f / T, 2));
    sc.set_target(velocity_a, 0);
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    xLastWakeTime = xTaskGetTickCount();
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
}

void straight_test() {
  if (!ui.waitForCover())
    return;
  delay(1000);
  bz.play(Buzzer::SELECT);
  imu.calibration();
  sc.enable();
  fan.drive(0.5);
  delay(500);
  sc.position.x = 0;
  straight_x(12 * 90 - 3 - MACHINE_TAIL_LENGTH, 2400, 0);
  sc.set_target(0, 0);
  delay(100);
  fan.drive(0);
  delay(500);
  sc.disable();
}

void log_test() {
  if (!ui.waitForCover())
    return;
  delay(1000);
  logstream.flush();
  auto printLog = []() {
    logstream << enc.position(0) << ",";
    logstream << enc.position(1) << ",";
    logstream << imu.gyro.z << ",";
    logstream << imu.accel.y << ",";
    logstream << std::endl;
  };
  bz.play(Buzzer::SELECT);
  imu.calibration();
  bz.play(Buzzer::CANCEL);
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (int i = 0; i < 500; i++) {
    mt.drive(i, i);
    printLog();
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
  }
  mt.drive(0, 0);
  vTaskDelay(500 / portTICK_RATE_MS);
  mt.free();
}

void petitcon() {
  if (!ui.waitForCover())
    return;
  delay(500);
  //      fr.set_path("sssssssrlrlrlrlrlrlssssslrlrlrlrlrlrsssssrlrlrlrlrlrssssssssrlrlrlrlrsssssssssssssssslrlrlrlrlsssssssslrlrlrlrlssssslrlrlrlrlrlrsssssrlrlrlrlrlrlssssss");
  fr.set_path("sssssssrlrlrlrlrlrlssssslrlrlrlrlrlrsssssrlrlrlrlrlrssssssssrl"
              "rlrlrlrlrssssssssssssssssssslrlrlrlrlrlsssssssslrlrlrlrlrlssss"
              "slrlrlrlrlrlrsssssrlrlrlrlrlrlssssss");
  //      fr.set_path("ssssssssrlrrlrssssssss");
  imu.calibration();
  fr.enable();
  fr.waitForEnd();
  fr.disable();
}

void normal_drive() {
  int mode = ui.waitForSelect(16);
  switch (mode) {
  //* 走行
  case 0:
    bz.play(Buzzer::SUCCESSFUL);
    if (!ui.waitForCover())
      return;
    led = 9;
    ms.start();
    btn.flags = 0;
    while (ms.isRunning() && !btn.pressed)
      delay(100);
    delay(1000);
    bz.play(Buzzer::CANCEL);
    btn.flags = 0;
    ms.terminate();
    break;
  //* 走行パラメータの選択 & 走行
  case 1: {
    int preset = ui.waitForSelect(16);
    if (preset < 0)
      return;
    switch (preset) {
    case 0: fr.runParameter = FastRun::RunParameter(0.7, 1200, 3000, 3000); break;
    case 1: fr.runParameter = FastRun::RunParameter(0.7, 1500, 3600, 3600); break;
    case 2: fr.runParameter = FastRun::RunParameter(0.7, 1800, 4500, 4500); break;
    case 3: fr.runParameter = FastRun::RunParameter(0.7, 2100, 6000, 6000); break;
    case 4: fr.runParameter = FastRun::RunParameter(0.8, 1200, 3000, 3000); break;
    case 5: fr.runParameter = FastRun::RunParameter(0.8, 1500, 3600, 3600); break;
    case 6: fr.runParameter = FastRun::RunParameter(0.8, 1800, 4500, 4500); break;
    case 7: fr.runParameter = FastRun::RunParameter(0.8, 2100, 6000, 6000); break;
    case 8: fr.runParameter = FastRun::RunParameter(0.9, 1200, 3000, 3000); break;
    case 9: fr.runParameter = FastRun::RunParameter(0.9, 1500, 3600, 3600); break;
    case 10: fr.runParameter = FastRun::RunParameter(0.9, 1800, 4500, 4500); break;
    case 11: fr.runParameter = FastRun::RunParameter(0.9, 2100, 6000, 6000); break;
    case 12: fr.runParameter = FastRun::RunParameter(1.0, 1200, 3000, 3000); break;
    case 13: fr.runParameter = FastRun::RunParameter(1.0, 1500, 3600, 3600); break;
    case 14: fr.runParameter = FastRun::RunParameter(1.0, 1800, 4500, 4500); break;
    case 15: fr.runParameter = FastRun::RunParameter(1.0, 2100, 6000, 6000); break;
    }
  }
    bz.play(Buzzer::SUCCESSFUL);
    break;
  //* 速度の設定
  case 2: {
    int value;
    for (int i = 0; i < 1; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    const float curve_gain = 0.1f * value;
    for (int i = 0; i < 2; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    const float v_max = 300.0f * value;
    for (int i = 0; i < 3; i++)
      bz.play(Buzzer::SHORT);
    value = ui.waitForSelect(16);
    if (value < 0)
      return;
    const float accel = 1000.0f * value;
    fr.runParameter = FastRun::RunParameter(curve_gain, v_max, accel, accel);
  }
    bz.play(Buzzer::SUCCESSFUL);
    break;
  //* 壁制御の設定
  case 3: {
    int value = ui.waitForSelect(16);
    if (value < 0)
      return;
    fr.wallAvoidFlag = value & 0x01;
    fr.wallAvoid45Flag = value & 0x02;
    fr.wallCutFlag = value & 0x04;
    fr.V90Enabled = value & 0x08;
  }
    bz.play(Buzzer::SUCCESSFUL);
    break;
  //* ファンの設定
  case 4: {
    fan.drive(fr.fanDuty);
    delay(100);
    fan.drive(0);
    int value = ui.waitForSelect(11);
    if (value < 0)
      return;
    fr.fanDuty = 0.1f * value;
    fan.drive(fr.fanDuty);
    ui.waitForSelect(1);
    fan.drive(0);
  }
    bz.play(Buzzer::SUCCESSFUL);
    break;
  //* 宴会芸
  case 5:
    position_test();
    break;
  //* 迷路データの復元
  case 7:
    bz.play(Buzzer::MAZE_RESTORE);
    if (!ms.restore()) {
      bz.play(Buzzer::ERROR);
      return;
    }
    bz.play(Buzzer::SUCCESSFUL);
    break;
  //* 前壁キャリブレーション
  case 8:
    led = 6;
    if (!ui.waitForCover(true))
      return;
    delay(1000);
    bz.play(Buzzer::CONFIRM);
    wd.calibrationFront();
    bz.play(Buzzer::CANCEL);
    break;
  //* 横壁キャリブレーション
  case 9:
    led = 9;
    if (!ui.waitForCover())
      return;
    delay(1000);
    bz.play(Buzzer::CONFIRM);
    wd.calibrationSide();
    bz.play(Buzzer::CANCEL);
    break;
  //* 前壁補正データの保存
  case 10:
    if (!ui.waitForCover())
      return;
    if (wd.backup()) {
      bz.play(Buzzer::SUCCESSFUL);
    } else {
      bz.play(Buzzer::ERROR);
    }
    break;
  //* ゴール区画の設定
  case 11: {
    for (int i = 0; i < 2; i++)
      bz.play(Buzzer::SHORT);
    int value = ui.waitForSelect(4);
    if (value < 0)
      return;
    switch (value) {
    case 0: {
      int x = ui.waitForSelect(16);
      int y = ui.waitForSelect(16);
      ms.set_goal({Vector(x, y)});
      break;
    }
    case 1:
      ms.set_goal({Vector(1, 0)});
      break;
    case 2:
      ms.set_goal({Vector(4, 4), Vector(4, 5), Vector(5, 4), Vector(5, 5)});
      break;
    case 3:
      ms.set_goal({Vector(15, 15)});
      break;
    }
    bz.play(Buzzer::SUCCESSFUL);
  } break;
  //* マス直線
  case 12:
    if (!ui.waitForCover(true))
      return;
    delay(1000);
    bz.play(Buzzer::CONFIRM);
    imu.calibration();
    bz.play(Buzzer::CANCEL);
    sc.enable();
    straight_x(9 * 90 - 6 - MACHINE_TAIL_LENGTH, 300, 0);
    sc.disable();
    break;
    //* テスト
  case 13:
    // log_test();
    // trapizoid_test();
    accel_test();
    // straight_test();
    // petitcon();
    break;
  //* ログの表示
  case 14:
    std::cout << logstream.str();
    break;
  //* リセット
  case 15:
    ms.print();
    ESP.restart();
    break;
  }
}

void mainTask(void *arg) {
  while (1) {
    normal_drive();
    delay(1);
  }
}
