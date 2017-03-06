/*

   Sassor ELP with ESP32

   Author:  kerikun11 (Github: kerikun11)
   Date:    2017.02.24

*/

#include <WiFi.h>
#include <SPI.h>
#include "esp_deep_sleep.h"
#include "esp_task_wdt.h"

#include "buzzer.h"
#include "mpu6500.h"

/* debug print settings */
#if 1
#define print_dbg               Serial.print
#define printf_dbg              Serial.printf
#define println_dbg             Serial.println
#else
#define print_dbg(...)          // No Operation
#define printf_dbg(...)         // No Operation
#define println_dbg(...)        // No Operation
#endif

#define BUTTON_PIN          0
#define BAT_VOL_PIN         36
#define LED_L_PIN           5
#define LED_R_PIN           2
#define PR_TX_SL_FR_PIN     16
#define PR_TX_SR_FL_PIN     17
#define PR_RX_SL_PIN        12
#define PR_RX_FL_PIN        13
#define PR_RX_FR_PIN        32
#define PR_RX_SR_PIN        33
#define FAN_PIN             15

#define LEDC_PR_CH          1
#define LEDC_FAN_CH         2

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
    delay(1000);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    //    esp_deep_sleep_enable_timer_wakeup(10 * 1000 * 1000);  // wakeup(restart) after 10secs
    esp_deep_sleep_start();
  }
  bz.play(Buzzer::BOOT);

  //  ledcSetup(LEDC_PR_CH, 10000, 4);
  //  ledcAttachPin(PR_TX_SL_FR_PIN, LEDC_PR_CH);
  //  ledcWrite(LEDC_PR_CH, 8);

  //  ledcSetup(LEDC_FAN_CH, 880, 8);
  //  ledcAttachPin(FAN_PIN, LEDC_FAN_CH);

  mpu.init();

  //  SPI.begin(MPU6500_SCLK_PIN, MPU6500_MISO_PIN, MPU6500_MOSI_PIN);
  //  digitalWrite(MPU6500_CS_PIN, HIGH);
  //  pinMode( MPU6500_CS_PIN, OUTPUT);
  //  SPI.beginTransaction(SPISettings(1000, SPI_MSBFIRST, SPI_MODE0));
  //  SPI.transfer(0x00 | 0x19);
  //  SPI.transfer(0x00 | 0x07);
  //  SPI.endTransaction();
}

void loop() {
  //  digitalWrite(LED_L_PIN, HIGH);
  //  digitalWrite(LED_R_PIN, LOW);
  //  delay(500);
  //  digitalWrite(LED_L_PIN, LOW);
  //  digitalWrite(LED_R_PIN, HIGH);
  //  delay(500);
  //  printf("%d\n", analogRead(PR_RX_FR_PIN));
  //  delayMicroseconds(10);
  //  printf("Battery Voltage: %.3f\n", 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095);
  //  delay(100);
  //  static bool state = false;
  //  static bool fan = false;
  //  if (state && digitalRead(BUTTON_PIN) == LOW) {
  //    delay(10);
  //    if (fan) {
  //      ledcWrite(LEDC_FAN_CH, 0x00);
  //      fan = false;
  //    } else {
  //      ledcWrite(LEDC_FAN_CH, 0xFF);
  //      fan = true;
  //    }
  //    state = false;
  //  } else {
  //    state = true;
  //  }
  //  printf("%d\t%d\t%d\t%d\n", analogRead(PR_RX_SL_PIN), analogRead(PR_RX_FL_PIN), analogRead(PR_RX_FR_PIN), analogRead(PR_RX_SR_PIN));
  mpu.print();
  //  digitalWrite(MPU6500_CS_PIN, LOW);
  //  SPI.beginTransaction(SPISettings(1000000, SPI_MSBFIRST, SPI_MODE0));
  //  SPI.transfer(0x80 | 0x3B);
  //  for (int i = 0; i < 7; i++) printf("%d\t", SPI.transfer16(0xFFFF));
  //  printf("\n");
  //  SPI.endTransaction();
  //  digitalWrite(MPU6500_CS_PIN, HIGH);
  delay(100);
}

