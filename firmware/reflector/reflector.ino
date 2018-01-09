/*
  KERISE
  Author:  kerikun11 (Github: kerikun11)
*/

#include <WiFi.h>

#define BAT_VOL_PIN             35
#define PR_TX_PINS              {12, 13, 12, 13}
#define PR_RX_PINS              {36, 38, 39, 37}

#include "reflector.h"
Reflector ref(PR_TX_PINS, PR_RX_PINS);

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
  Serial.begin(2000000);
  log_i("KERISE Reflector Sample");
  batteryCheck();

  ref.begin();
  delay(1000);
  xTaskCreate(task, "test", 4096, NULL, 0, NULL);
}

void task(void* arg) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    ref.csv(); vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
  }
}

void loop() {
  delay(1000);
}

