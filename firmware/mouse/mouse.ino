/*
  KERISE v3
  Author:  kerikun11 (Github: kerikun11)
  Date:    2017.02.24
*/

#include <WiFi.h>
#include "esp_deep_sleep.h"

#include "as5145.h"
#include "buzzer.h"
#include "motor.h"
#include "mpu6500.h"
#include "reflector.h"

AS5145 as;
Buzzer bz(BUZZER_PIN, LEDC_BUZZER_CH);
Motor mt;
MPU6500 mpu;
Reflector ref;

void setup() {
  WiFi.mode(WIFI_OFF);
  printf("\n****** KERISE v3 ******\n");
  pinMode(LED_L_PIN, OUTPUT);
  pinMode(LED_R_PIN, OUTPUT);

  bz.init();
  float voltage = 3.02656f * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
  printf("Battery Voltage: %.3f\n", voltage);
  if (voltage < 3.8f) {
    printf("Battery Low!\n");
    bz.play(Buzzer::LOW_BATTERY);
    delay(2000);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    //    esp_deep_sleep_enable_timer_wakeup(10 * 1000 * 1000);  // wakeup(restart) after 10secs
    esp_deep_sleep_start();
  }
  bz.play(Buzzer::BOOT);

  //  ledcSetup(LEDC_PR_CH, 2000, 4);
  //  ledcAttachPin(PR_TX_SL_FR_PIN, LEDC_PR_CH);
  //  ledcWrite(LEDC_PR_CH, 1);

  //  mpu.init();
  //  mpu.calibration();
  //  as.init();
  ref.init();
}

void loop() {
  //  mpu.print();
  //  as.print();
  ref.print();
  delay(100);
}
