/**
  KERISE v3
  Author:  kerikun11 (Github: kerikun11)
  Date:    2017.02.24
*/

#include <WiFi.h>
#include <FS.h>
#include "config.h"

/* Hardware */
#include "UserInterface.h"
#include "motor.h"
#include "icm20602.h"
#include "as5048a.h"
#include "reflector.h"
#include "ToF.h"

Buzzer bz(BUZZER_PIN, LEDC_CH_BUZZER);
Button btn(BUTTON_PIN);
LED led(LED_PINS);
Motor mt;
Fan fan;
ICM20602 icm;
AS5048A as;
Reflector ref(PR_TX_PINS, PR_RX_PINS);
ToF tof(TOF_SDA_PIN, TOF_SCL_PIN);

/* Software */
#include "Emergency.h"
#include "debug.h"
#include "logger.h"
#include "BLETransmitter.h"
#include "WallDetector.h"
#include "SpeedController.h"
#include "FastRun.h"
#include "SearchRun.h"
#include "MazeSolver.h"

Emergency em;
ExternalController ec;
Logger lg;
BLETransmitter ble;
WallDetector wd;
SpeedController sc;
FastRun fr;
SearchRun sr;
MazeSolver ms;

void batteryCheck() {
  float voltage = 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
  printf("Battery Voltage: %.3f\n", voltage);
  if (voltage < 3.8f) {
    printf("Battery Low!\n");
    bz.play(Buzzer::LOW_BATTERY);
    delay(3000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
  }
}

void setup() {
  WiFi.mode(WIFI_OFF);
  Serial.begin(115200);
  pinMode(RX, INPUT_PULLUP);
  printf("\n************ KERISE v3-2 ************\n");
  printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  led = 0xf;

  batteryCheck();
  bz.play(Buzzer::BOOT);

  icm.begin(true);
  as.begin(false);
  em.init();
  ec.init();
  ref.begin();
  tof.begin();
  wd.begin();
  //  ble.begin();

  //  lg.start();
}

void loop() {
#define TEST 0
#if TEST == 0
  normal_drive();
#elif TEST == 1
  trapizoid_test();
#elif TEST == 2
  fan_test();
#elif TEST == 3
  straight_test();
#elif TEST == 4
  ref.csv();
  delay(10);
#elif TEST == 5
  printf("0,1800,%d,%d\n", ref.read(Reflector::REF_CH_FL, 1), ref.read(Reflector::REF_CH_FR, 1));
  delay(10);
#elif TEST == 6
  wd.print();
  delay(100);
#else
  icm.print();
  as.print();
  delay(100);
  //  printf("%f,%f\n", as.position(0), as.position(1));
  //  printf("%d,%d\n", as.getPulses(0), as.getPulses(1));
  //  printf("%d,%d\n", as.getRaw(0), as.getRaw(1));
#endif
}

void normal_drive() {
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
}

void trapizoid_test() {
  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    delay(1000);
    icm.calibration();
    bool suction = true;
    if (suction) fan.drive(0.5);
    delay(500);
    lg.start();
    sc.enable(suction);
    const float accel = 12000;
    const float decel = 12000;
    const float v_max = 2400;
    const float v_start = 0;
    float T = 1.5f * (v_max - v_start) / accel;
    for (int ms = 0; ms / 1000.0f < T; ms++) {
      float velocity_a = v_start + (v_max - v_start) * 6.0f * (-1.0f / 3 * pow(ms / 1000.0f / T, 3) + 1.0f / 2 * pow(ms / 1000.0f / T, 2));
      sc.set_target(velocity_a, 0);
      delay(1);
    }
    bz.play(Buzzer::SELECT);
    delay(100);
    bz.play(Buzzer::SELECT);
    for (float v = v_max; v > 0; v -= decel / 1000) {
      sc.set_target(v, 0);
      delay(1);
    }
    sc.set_target(0, 0);
    delay(300);
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

void fan_test() {
  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::CANCEL);
    fan.drive(0);
  }
  if (btn.long_pressed_1) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    fan.drive(0.4);
  }
}

void straight_test() {
  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    delay(1000);
    icm.calibration();
    sc.enable(true);
    lg.start();
    sc.getPosition().reset();
    fan.drive(0.5);
    delay(500);
    straight_x(800, 1500, 0);
    sc.set_target(0, 0);
    fan.drive(0);
    delay(100);
    sc.disable();
    lg.end();
  }
  if (btn.long_pressing_1) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    lg.print();
  }
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
      led = 3;
      ms.start();
      break;
    case 1: {
        int speed = waitForSelect(8);
        fr.fast_speed = 200 + 200 * speed;
      }
      if (!waitForCover()) return;
      break;
    case 2: {
        int gain = waitForSelect(8);
        fr.fast_curve_gain = 0.1f + 0.1f * gain;
      }
      if (!waitForCover()) return;
      break;
    case 3:
      if (!waitForCover()) return;
      ms.set_goal({Vector(1, 0)});
      bz.play(Buzzer::CONFIRM);
      break;
  }
  bz.play(Buzzer::SELECT);
}

bool waitForCover() {
  while (1) {
    delay(1);
    if (ref.front(0) > 120 && ref.front(1) > 120) {
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

Position getRelativePosition() {
  return sc.getPosition();
}

#define TEST_LOOK_AHEAD 20
#define TEST_PROP_GAIN  20

void straight_x(const float distance, const float v_max, const float v_end) {
  const float accel = 3000;
  const float decel = 3000;
  portTickType xLastWakeTime = xTaskGetTickCount();
  int ms = 0;
  const float v_start = sc.actual.trans;
  const float T = 1.5f * (v_max - v_start) / accel;
  while (1) {
    Position cur = getRelativePosition();
    if (v_end >= 1.0f && cur.x > distance - TEST_LOOK_AHEAD) break;
    if (v_end < 1.0f && cur.x > distance - 1.0f) break;
    float extra = distance - cur.x;
    float velocity_a = v_start + (v_max - v_start) * 6.0f * (-1.0f / 3 * pow(ms / 1000.0f / T, 3) + 1.0f / 2 * pow(ms / 1000.0f / T, 2));
    float velocity_d = sqrt(2 * decel * fabs(extra) + v_end * v_end);
    float velocity = v_max;
    if (velocity > velocity_d) velocity = velocity_d;
    if (ms / 1000.0f < T && velocity > velocity_a) velocity = velocity_a;
    float theta = atan2f(-cur.y, TEST_LOOK_AHEAD * (1 + velocity / 600.0f)) - cur.theta;
    sc.set_target(velocity, TEST_PROP_GAIN * theta);
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    ms++;
  }
  sc.set_target(v_end, 0);
}

