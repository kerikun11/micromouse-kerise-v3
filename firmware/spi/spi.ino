/*
  KERISE
  Author:  kerikun11 (Github: kerikun11)
*/

#include <WiFi.h>

#define BAT_VOL_PIN             35

#define AS5048A_MOSI_PIN        23
#define AS5048A_MISO_PIN        19
#define AS5048A_SCLK_PIN        18
#define AS5048A_CS_PIN          15
#define AS5048A_SPI_HOST        HSPI_HOST
#define AS5048A_SPI_DMA_CHAIN   1

#define ICM20602_MOSI_PIN       23
#define ICM20602_MISO_PIN       19
#define ICM20602_SCLK_PIN       18
#define ICM20602_CS_PIN         14
#define ICM20602_SPI_HOST       HSPI_HOST
#define ICM20602_SPI_DMA_CHAIN  1

#define MACHINE_ROTATION_RADIUS 16.7f
#define MACHINE_GEAR_RATIO      (1.0f/3.0f) //< 10/30
#define MACHINE_WHEEL_DIAMETER  13.28f
#define MACHINE_TAIL_LENGTH     18.4f

#include "axis.h"
#include "encoder.h"

Axis axis;
Encoder enc;

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
  log_i("KERISE v3-2");
  batteryCheck();

  axis.begin(true);
  enc.begin(false);
  //  delay(2000);
  //  axis.calibration();
  xTaskCreate(task, "test", 4096, NULL, 0, NULL);
}

void task(void* arg) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);
    //    enc.csv();
    //    printf("0,%f,%f,%f\n", PI, -PI, axis.gyro.z * 1000);
    //    printf("0,%f,%f,%f,%f,%f\n", 9806.65f, -9806.65f, axis.accel.x, axis.accel.y, axis.accel.z);
    //    printf("0,%f,%f,%f\n", PI, -PI, axis.gyro.z * 100);
  }
}

void loop() {
  axis.print();
  //  enc.print();
  delay(100);
  if (Serial.available()) {
    switch (Serial.read()) {
      case 't':
        axis.calibration();
        break;
      default:
        break;
    }
  }
}

