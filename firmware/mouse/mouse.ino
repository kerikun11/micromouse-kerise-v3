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

//#define printf lg.printf

void task(void* arg) {
  portTickType xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    //        ref.csv(); vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    //    tof.csv();vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    //    ref.print(); vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
    //    wd.print(); vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
    //    const int i = 0;
    //    printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", sc.target.trans, sc.actual.trans, sc.enconly.trans, sc.Kp * sc.proportional.trans, sc.Ki * sc.integral.trans, sc.Kd * sc.differential.trans, sc.Kp * sc.proportional.trans + sc.Ki * sc.integral.trans + sc.Kd * sc.differential.trans);
    //    printf("0,%f,%f,%f\n", PI, -PI, axis.gyro.z * 10);
    //    printf("0,%f,%f,%f\n", PI, -PI, axis.angle.z * 10);
    //    printf("0,%f,%f,%f,%f,%f\n", PI, -PI, axis.angle.x * 10, axis.angle.y * 10, axis.angle.z * 10);
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

void loop() {
  normal_drive();
  //  position_test();
  //  trapizoid_test();
  //  straight_test();
  //  turn_test();
}

const int searchig_time_ms = 3 * 60 * 1000;
//const int searchig_time_ms = 1 * 60 * 1000;
bool timeup = false;

void normal_drive() {
  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    if (ms.isRunning()) ms.terminate();
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

void position_test() {
  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    //    delay(1000);
    //    bz.play(Buzzer::SELECT);
    //    axis.calibration();
    //    bz.play(Buzzer::CANCEL);
    //        sc.enable();
    //        sc.set_target(0, 0);
    mt.drive(200, 200);
    delay(2000);
    mt.drive(0, 0);
  }
  if (btn.long_pressing_1) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    lg.print();
  }
  delay(10);
}

void trapizoid_test() {
  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
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
  if (btn.long_pressing_1) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    lg.print();
  }
}

void straight_test() {
  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    delay(2000);
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
  if (btn.long_pressing_1) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    lg.print();
  }
}

void turn_test() {
  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    delay(1000);
    axis.calibration();
    bz.play(Buzzer::CONFIRM);
    lg.start();
    sc.enable();
    turn(PI / 2);
    sc.disable();
    bz.play(Buzzer::CANCEL);
    lg.end();
  }
  if (btn.long_pressing_1) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    lg.print();
  }
}

#define TEST_LOOK_AHEAD 12
#define TEST_PROP_GAIN  20

void straight_x(const float distance, const float v_max, const float v_end) {
  const float accel = 1500;
  const float decel = 1200;
  int ms = 0;
  const float v_start = sc.actual.trans;
  const float T = 1.5f * (v_max - v_start) / accel;
  portTickType xLastWakeTime = xTaskGetTickCount();
  while (1) {
    Position cur = sc.position;
    if (v_end >= 1.0f && cur.x > distance - TEST_LOOK_AHEAD) break;
    if (v_end < 1.0f && cur.x > distance - 1.0f) break;
    float extra = distance - cur.x;
    float velocity_a = v_start + (v_max - v_start) * 6.0f * (-1.0f / 3 * pow(ms / 1000.0f / T, 3) + 1.0f / 2 * pow(ms / 1000.0f / T, 2));
    float velocity_d = sqrt(2 * decel * fabs(extra) + v_end * v_end);
    float velocity = v_max;
    if (velocity > velocity_d) velocity = velocity_d;
    if (ms / 1000.0f < T && velocity > velocity_a) velocity = velocity_a;
    float theta = atan2f(-cur.y, TEST_LOOK_AHEAD * (1 + velocity / 600)) - cur.theta;
    sc.set_target(velocity, TEST_PROP_GAIN * theta);
    //    if (avoid) wall_avoid();
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    ms++;
    printf("%f\t[%c %c]\n", cur.x + 6 + MACHINE_TAIL_LENGTH, wd.wall[0] ? 'X' : '_', wd.wall[1] ? 'X' : '_');
  }
  sc.set_target(v_end, 0);
}

void turn(const float angle) {
  const float speed = 3 * M_PI;
  const float accel = 36 * M_PI;
  const float decel = 12 * M_PI;
  const float back_gain = 10.0f;
  int ms = 0;
  portTickType xLastWakeTime = xTaskGetTickCount();
  while (1) {
    if (fabs(sc.actual.rot) > speed) break;
    float delta = sc.position.x * cos(-sc.position.theta) - sc.position.y * sin(-sc.position.theta);
    if (angle > 0) {
      sc.set_target(-delta * back_gain, ms / 1000.0f * accel);
    } else {
      sc.set_target(-delta * back_gain, -ms / 1000.0f * accel);
    }
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    ms++;
  }
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    float extra = angle - sc.position.theta;
    if (fabs(sc.actual.rot) < 0.1 && fabs(extra) < 0.1) break;
    float target_speed = sqrt(2 * decel * fabs(extra));
    float delta = sc.position.x * cos(-sc.position.theta) - sc.position.y * sin(-sc.position.theta);
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

void batteryCheck() {
  float voltage = 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
  printf("Battery Voltage: %.3f\n", voltage);
  led = (uint8_t)((voltage - 3.6f) / (4.4f - 3.6f) * 16);
  if (voltage < 3.8f) {
    printf("Battery Low!\n");
    bz.play(Buzzer::LOW_BATTERY);
    delay(3000);
    led = 0;
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
  }
}

int waitForSelect(int range = 16) {
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
    if (btn.pressed) {
      btn.flags = 0;
      bz.play(Buzzer::CANCEL);
      log_i("waitForCover() => true");
      return false;
    }
  }
}

bool waitForFix() {
  int fix_count = 0;
  while (1) {
    delay(1);
    if (fabs(axis.gyro.x) < 0.01f * PI && fabs(axis.gyro.y) < 0.01f * PI && fabs(axis.gyro.z) < 0.01f * PI) {
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

bool restore() {
  size_t n = pref.getBytes("wd.wall_ref", &(wd.wall_ref), sizeof(WallDetector::WallValue));
  if (n == 0) {
    log_e("Restore Failed:(");
    return false;
  }
  log_i("Restore Successful");
  log_d("wd.wall_ref: %d, %d, %d, %d", wd.wall_ref.side[0], wd.wall_ref.front[0], wd.wall_ref.front[1], wd.wall_ref.side[1]);
  return true;
}

bool backup() {
  size_t n = pref.putBytes("wd.wall_ref", &(wd.wall_ref), sizeof(WallDetector::WallValue));
  if (n == 0) {
    log_e("Backup Failed:(");
    return false;
  }
  log_i("Backup Successful");
  return true;
}

void task() {
  int mode = waitForSelect(16);
  switch (mode) {
    case 0:
      if (!waitForCover()) return;
      led = 9;
      ms.start(timeup);
      break;
    case 1: {
        int preset = waitForSelect(8);
        if (preset < 0) break;
        if (!waitForCover()) return;
        switch (preset) {
          case 0: fr.runParameter = FastRun::RunParameter(0.1, 100, 1200, 900); break;
          case 1: fr.runParameter = FastRun::RunParameter(0.3, 400, 1500, 1200); break;
          case 2: fr.runParameter = FastRun::RunParameter(0.5, 900, 2000, 1500); break;
          case 3: fr.runParameter = FastRun::RunParameter(0.5, 1200, 3000, 2000); break;
          case 4: fr.runParameter = FastRun::RunParameter(0.6, 1200, 3000, 2000); break;
          case 5: fr.runParameter = FastRun::RunParameter(0.6, 1500, 6000, 3000); break;
          case 6: fr.runParameter = FastRun::RunParameter(0.7, 1800, 9000, 6000); break;
        }
      }
      bz.play(Buzzer::COMPLETE);
      break;
    case 3:
      bz.play(Buzzer::MAZE_RESTORE);
      if (!waitForCover()) return;
      if (ms.restore()) {
        bz.play(Buzzer::COMPLETE);
      } else {
        bz.play(Buzzer::ERROR);
      }
      break;
    case 4:
      bz.play(Buzzer::MAZE_BACKUP);
      if (!waitForCover()) return;
      if (backup()) {
        bz.play(Buzzer::COMPLETE);
      } else {
        bz.play(Buzzer::ERROR);
      }
      break;
    case 5:
      bz.play(Buzzer::SHORT);
      if (!waitForCover()) return;
      ms.set_goal({Vector(1, 0)});
      bz.play(Buzzer::COMPLETE);
      break;
    case 6:
      for (int i = 0; i < 2; i++) bz.play(Buzzer::SHORT);
      if (!waitForCover()) return;
      ms.set_goal({Vector(4, 4), Vector(4, 5), Vector(5, 4), Vector(5, 5)});
      bz.play(Buzzer::COMPLETE);
      break;
    case 7:
      for (int i = 0; i < 3; i++) bz.play(Buzzer::SHORT);
      if (!waitForCover()) return;
      ms.set_goal({Vector(3, 3), Vector(3, 4), Vector(3, 5), Vector(4, 3), Vector(4, 4), Vector(4, 5), Vector(5, 3), Vector(5, 4), Vector(5, 5)});
      bz.play(Buzzer::COMPLETE);
      break;
    case 8:
      if (!waitForCover(true)) return;
      delay(1000);
      bz.play(Buzzer::CONFIRM);
      wd.calibrationFront();
      bz.play(Buzzer::CANCEL);
      break;
    case 9:
      if (!waitForCover()) return;
      delay(1000);
      bz.play(Buzzer::CONFIRM);
      wd.calibration();
      bz.play(Buzzer::CANCEL);
      break;
  }
}

