/*
  KERISE
  Author:  kerikun11 (Github: kerikun11)
*/

#include <WiFi.h>

#define BAT_VOL_PIN         35
#define ICM20602_CS_PIN     14
#define ICM20602_SPI_HOST   HSPI_HOST
#define AS5048A_CS_PIN      15
#define AS5048A_SPI_HOST    HSPI_HOST

#define SPI_MOSI_PIN        23
#define SPI_MISO_PIN        19
#define SPI_SCLK_PIN        18
#define SPI_HOST_SEL        HSPI_HOST
#define SPI_DMA_CHAIN       1

#define MACHINE_ROTATION_RADIUS 16.7f
#define MACHINE_GEAR_RATIO      (1.0f/3.0f) //< 10/30
#define MACHINE_WHEEL_DIAMETER  13.28f
#define MACHINE_TAIL_LENGTH     18.4f

#include "icm20602.h"
#include "as5048a.h"

ICM20602 icm;
AS5048A as;

void setup() {
  WiFi.mode(WIFI_OFF);
  Serial.begin(115200);
  log_i("KERISE v3-2");

  icm.begin(true);
  as.begin(false);
  delay(2000);
  icm.calibration();
  xTaskCreate(task, "test", 4096, NULL, 0, NULL);
}

void task(void* arg) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);
    as.csv();
    //    printf("0,%f,%f,%f\n", PI, -PI, icm.gyro.z * 1000);
  }
}

void loop() {
  //  icm.print();
  //  as.print();
  delay(100);
  if (Serial.available()) {
    switch (Serial.read()) {
      case 't':
        icm.calibration();
        break;
      default:
        break;
    }
  }
}

