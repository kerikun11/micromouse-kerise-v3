/*
  KERISE
  Author:  kerikun11 (Github: kerikun11)
*/

#include <WiFi.h>
#include "config.h"

#include "motor.h"
Motor mt;

#define BAT_VOL_PIN         35

void batteryCheck() {
  float voltage = 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
  printf("Battery Voltage: %.3f\n", voltage);
  if (voltage < 3.8f) {
    printf("Battery Low!\n");
    delay(3000);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
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
  delay(3000);
  printf("free\n");
  mt.free();
}

void task(void* arg) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
  }
}

void loop() {
  delay(100);
}

