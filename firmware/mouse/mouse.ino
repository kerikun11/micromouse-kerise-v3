/*
  KERISE v3
  Author:  kerikun11 (Github: kerikun11)
  Date:    2017.02.24
*/

#include <WiFi.h>
#include "esp_deep_sleep.h"

#include "as5145.h"
#include "UserInterface.h"
#include "motor.h"
#include "mpu6500.h"
#include "reflector.h"

extern AS5145 as;
extern Buzzer bz;
extern Button btn;
extern LED led;
extern Motor mt;
extern MPU6500 mpu;
extern Reflector ref;

AS5145 as;
Buzzer bz(BUZZER_PIN, LEDC_BUZZER_CH);
Button btn(BUTTON_PIN);
LED led(LED_L_PIN, LED_R_PIN);
Motor mt;
MPU6500 mpu;
Reflector ref;

void setup() {
  WiFi.mode(WIFI_OFF);
  printf("\n************ KERISE v3 ************\n");
  led = 3;

  bz.init();
  btn.init();
  //  float voltage = 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
  float voltage = 3.02656f * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
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

  //  mpu.init();
  //  mpu.calibration();
  //  as.init();
  //  ref.init();
}

void loop() {
  mpu.print();
  as.print();
  ref.print();
  delay(100);
}

