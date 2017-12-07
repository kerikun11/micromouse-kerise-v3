/*
  KERISE
  Author:  kerikun11 (Github: kerikun11)
*/

#include <WiFi.h>

/* Hardware Mapping */
#define BAT_VOL_PIN             35

#define MOTOR_L_CTRL1_PIN       25 // 16
#define MOTOR_L_CTRL2_PIN       26 // 17
#define MOTOR_R_CTRL1_PIN       16 // 25
#define MOTOR_R_CTRL2_PIN       17 // 26
#define FAN_PIN                 33

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

/* LEDC Channel */
#define LEDC_CH_MOTOR_L_CTRL1   0
#define LEDC_CH_MOTOR_L_CTRL2   1
#define LEDC_CH_MOTOR_R_CTRL1   2
#define LEDC_CH_MOTOR_R_CTRL2   3

#define LEDC_CH_FAN             6

/* Machine Size Parameter */
#define MACHINE_ROTATION_RADIUS 16.75f
#define MACHINE_GEAR_RATIO      (13.0f/41.0f)
#define MACHINE_WHEEL_DIAMETER  13.807967f
#define MACHINE_TAIL_LENGTH     18.4f

#include "motor.h"
#include "imu.h"
#include "encoder.h"

Motor mt;
Fan fan;
IMU imu;
Encoder enc;

#include "SpeedController.h"
SpeedController sc;

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
  log_i("KERISE v3-2");
  batteryCheck();

  imu.begin(true);
  enc.begin(false);
  xTaskCreate(task, "test", 4096, NULL, 0, NULL);
  sc.enable();
  delay(2000);
  imu.calibration();
}

void task(void* arg) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 2 / portTICK_RATE_MS);
    //    printf("0,-500,500,%d,%f,%f,%f,%f\n",
    //           sc.ave_num,
    //           sc.target.trans,
    //           sc.actual.trans,
    //           sc.enconly.trans,
    //           sc.acconly.trans
    //          );
    printf("%f,%f,%f,0,-100,100\n",
           sc.target.trans,
           sc.actual.trans,
           sc.enconly.trans
          );
  }
}

TaskHandle_t handle;

void loop() {
  delay(100);
  if (Serial.available()) {
    switch (Serial.read()) {
      case 't':
        break;
      case 's': {
          mt.drive(50, 50);
          delay(1000);
          portTickType xLastWakeTime = xTaskGetTickCount();
          for (int i = 0; i < 5000; i++) {
            vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
            printf("%d,%d\n", enc.getPulses(0), enc.getPulses(1));
          }
          mt.free();
        }
        break;
      case 'g':
        mt.drive(100, 100);
        break;
      case 'f':
        mt.free();
        break;
      default:
        break;
    }
  }
}

