#include "utils.h"

#include <WiFi.h>
#include <SPIFFS.h>
#include <Preferences.h>
extern Preferences pref;

#include "config.h"

/* Hardware */
#include "UserInterface.h"
#include "motor.h"
#include "axis.h"
#include "encoder.h"
#include "reflector.h"
#include "tof.h"

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
  const float back_gain = 5.0f;
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

void batteryLedIndicate(const float voltage) {
  if (voltage < 3.9f) led = 0x01;
  else if (voltage < 4.1f) led = 0x03;
  else if (voltage < 4.3f) led = 0x07;
  else led = 0x0F;
}

void batteryCheck() {
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

int waitForSelect(int range) {
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

bool waitForCover(bool side) {
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

