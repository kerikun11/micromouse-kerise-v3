/*
  KERISE
  Author:  kerikun11 (Github: kerikun11)
*/

#include <WiFi.h>

//#define BAT_VOL_PIN             36
//#define BUZZER_PIN              21
//#define LED_PINS                {5, 2}
//#define BUTTON_PIN              0
#define BAT_VOL_PIN             35
#define BUZZER_PIN              32
#define LED_PINS                {2, 4, 5, 27}
#define BUTTON_PIN              0

#define LEDC_CH_BUZZER          4

#include "UserInterface.h"
Buzzer bz(BUZZER_PIN, LEDC_CH_BUZZER);
Button btn(BUTTON_PIN);
LED led(LED_PINS);

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
  log_i("KERISE");

  batteryCheck();
  xTaskCreate(task, "test", 4096, NULL, 0, NULL);
}

void task(void* arg) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_RATE_MS);
    led = led + 1;
    bz.play(Buzzer::SHORT);
  }
}

void loop() {
  if (btn.pressed) {
    btn.flags = 0;
    bz.play(Buzzer::SELECT);
  }
  if (btn.long_pressed_1) {
    btn.flags = 0;
    bz.play(Buzzer::CONFIRM);
  }
}

