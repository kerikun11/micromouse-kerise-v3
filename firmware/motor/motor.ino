/*
  KERISE
  Author:  kerikun11 (Github: kerikun11)
*/

#include <WiFi.h>

//#define BAT_VOL_PIN             36
//#define MOTOR_L_CTRL1_PIN       18
//#define MOTOR_L_CTRL2_PIN       23
//#define MOTOR_R_CTRL1_PIN       19
//#define MOTOR_R_CTRL2_PIN       22
//#define FAN_PIN                 15
#define BAT_VOL_PIN             35
#define MOTOR_L_CTRL1_PIN       16
#define MOTOR_L_CTRL2_PIN       17
#define MOTOR_R_CTRL1_PIN       25
#define MOTOR_R_CTRL2_PIN       26
#define FAN_PIN                 33

#define LEDC_CH_MOTOR_L_CTRL1   0
#define LEDC_CH_MOTOR_L_CTRL2   1
#define LEDC_CH_MOTOR_R_CTRL1   2
#define LEDC_CH_MOTOR_R_CTRL2   3

#define LEDC_CH_FAN             6

#include "motor.h"
Motor mt;
Fan fan;

void batteryCheck() {
  float voltage = 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
  printf("Battery Voltage: %.3f\n", voltage);
  if (voltage < 3.8f) {
    printf("Battery Low!\n");
    //    bz.play(Buzzer::LOW_BATTERY);
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
  log_i("KERISE v3");

  batteryCheck();

  //  xTaskCreate(task, "test", 1024, NULL, 0, NULL);

  delay(1000);
  printf("drive\n");
  mt.drive(200, 200);
  fan.drive(0.5);
  delay(600);
  printf("free\n");
  mt.free();
  fan.drive(0);
}

void task(void* arg) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
  }
}

void loop() {
}

