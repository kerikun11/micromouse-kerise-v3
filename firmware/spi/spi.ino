/*
  KERISE
  Author:  kerikun11 (Github: kerikun11)
*/

#include <WiFi.h>
#include "config.h"

#include "icm20602.h"
#include "as5048a.h"

ICM20602 icm;
AS5048A as;

void setup() {
  WiFi.mode(WIFI_OFF);
  Serial.begin(115200);
  log_i("KERISE v3");

  // ESP-IDF SPI bus initialization
  spi_bus_config_t bus_cfg = {0};
  bus_cfg.mosi_io_num = ICM20602_MOSI_PIN;
  bus_cfg.miso_io_num = ICM20602_MISO_PIN;
  bus_cfg.sclk_io_num = ICM20602_SCLK_PIN;
  bus_cfg.quadwp_io_num = -1;
  bus_cfg.max_transfer_sz = 0; // defaults to 4094 if 0
  ESP_ERROR_CHECK(spi_bus_initialize(ICM20602_SPI, &bus_cfg, ICM20602_DMA_CHAIN));

  //  icm.begin();
  as.begin();
  delay(1000);
  //  icm.calibration();
  //  xTaskCreate(task, "test", 1024, NULL, 0, NULL);
}

void task(void* arg) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
  }
}

void loop() {
  //  icm.print();
  as.print();
  delay(100);
}

