/*
  KERISE v3
  Author:  kerikun11 (Github: kerikun11)
  Date:    2017.02.24
*/

#include <WiFi.h>
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

extern AS5145 as;
extern Buzzer bz;
extern Button btn;
extern LED led;
extern Emergency em;
extern ExternalController ec;
extern Logger lg;
extern Motor mt;
extern Fan fan;
extern MPU6500 mpu;
extern Reflector ref;
extern WallDetector wd;
extern SpeedController sc;
extern MoveAction ma;
extern MazeSolver ms;

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
  pinMode(RX, INPUT_PULLUP);
  Serial.begin(250000);
  printf("\n************ KERISE v3 ************\n");
  led = 3;

  bz.init();
  btn.init();
  float voltage = 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
  //  float voltage = 3.32656f * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
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

  //  WiFi.mode(WIFI_STA);
  //  WiFi.begin("WiFi-2.4GHz", "kashimamerda");
  //  printf("WiFi Connecting...");
  //  while (WiFi.status() != WL_CONNECTED) {
  //    delay(500);
  //    printf("wait...\n");
  //  }
  //  printf("WiFi connected");
  //  printf("IP address: %s", WiFi.localIP().toString().c_str());

  delay(500);
  em.init();
  ec.init();
  as.init();
  mpu.init();
  ref.init();
  //  mpu.calibration();

  wd.enable();
  wd.calibration();
}

bool en = false;

void loop() {
  //  mpu.print();
  //  as.print();
  //  printf("%f,%f\n", as.position(0), as.position(1));
  //  printf("%d,%d\n", as.getPulses(0), as.getPulses(1));
  //  printf("%d,%d\n", as.getRaw(0), as.getRaw(1));
  //  wd.print();
  //  delay(100);

  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
    delay(1000);
    mpu.calibration();
    fan.drive(0.8);
    delay(200);
    lg.start();
    sc.enable();
    sc.set_target(900, 0);
    delay(400);
    bz.play(Buzzer::SELECT);
    sc.set_target(0, 0);
    delay(200);
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

  //  if (btn.pressed) {
  //    btn.flags = 0;
  //    bz.play(Buzzer::CONFIRM);
  //    delay(500);
  //    ms.start();
  //  }
}

