/*
  KERISE
  Author:  kerikun11 (Github: kerikun11)
*/

#include <WiFi.h>

#define BAT_VOL_PIN             35

#include "BLETransmitter.h"
BLETransmitter ble;

void batteryCheck() {
  float voltage = 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
  printf("Battery Voltage: %.3f\n", voltage);
  if (voltage < 3.8f) {
    printf("Battery Low!\n");
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
  log_i("KERISE BLE Sample");
  //  batteryCheck();

  ble.begin();
  xTaskCreate(task, "test", 8192, NULL, 0, NULL);
}

void task(void* arg) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_RATE_MS);
    int us = micros();
    for (int i = 0; i < 10; i++) {
      ble.printf("micros(): %d", micros());
    }
    log_d("end:\t%ld", micros() - us);
  }
}

void loop() {
  delay(3000);
}

