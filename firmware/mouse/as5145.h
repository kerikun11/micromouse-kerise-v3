#pragma once

#include <Arduino.h>
#include "task_base.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "config.h"

#define AS5145_MISO_PIN   34
#define AS5145_SCLK_PIN   4
#define AS5145_CS_PIN     14

#define AS5145_SPI        HSPI_HOST
#define AS5145_DMA_CHAIN  1

#define AS5145_TASK_PRIORITY    1
#define AS5145_TASK_STACK_SIZE  512

class AS5145: private TaskBase {
  public:
    AS5145(): TaskBase("AS5145 Task", AS5145_TASK_PRIORITY, AS5145_TASK_STACK_SIZE) {
    }
    virtual ~AS5145() {
    }
    void init() {
      static spi_bus_config_t bus_cfg = {0};
      bus_cfg.sclk_io_num = AS5145_SCLK_PIN;
      bus_cfg.miso_io_num = AS5145_MISO_PIN;
      bus_cfg.mosi_io_num = -1;
      bus_cfg.quadhd_io_num = -1;
      bus_cfg.quadwp_io_num = -1;
      ESP_ERROR_CHECK(spi_bus_initialize(AS5145_SPI, &bus_cfg, AS5145_DMA_CHAIN));

      static spi_device_interface_config_t as5145_dev_cfg = {0};
      as5145_dev_cfg.flags = 0;
      as5145_dev_cfg.clock_speed_hz = 1000000;
      as5145_dev_cfg.mode = 4;
      as5145_dev_cfg.spics_io_num = AS5145_CS_PIN;
      as5145_dev_cfg.queue_size = 1;
      ESP_ERROR_CHECK(spi_bus_add_device(AS5145_SPI, &as5145_dev_cfg, &as5145_spi));

      create_task();
    }
    void print() {
      printf("L: %d\tR: %d\n", pulses[0], pulses[1]);
    }
  private:
    spi_device_handle_t as5145_spi;
    int pulses[2];
    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);

        static spi_transaction_t trans = {0};
        trans.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        trans.tx_data[0] = 0xAA;
        trans.tx_data[1] = 0xAA;
        trans.tx_data[2] = 0xAA;
        trans.tx_data[3] = 0xAA;
        trans.length = 32;
        ESP_ERROR_CHECK(spi_device_queue_trans(as5145_spi, &trans, portMAX_DELAY));

        static spi_transaction_t *rtrans;
        ESP_ERROR_CHECK(spi_device_get_trans_result(as5145_spi, &rtrans, portMAX_DELAY));
        pulses[0] = ((uint16_t) rtrans->rx_data[0] << 4) | (rtrans->rx_data[1] >> 4);
        pulses[1] = (0x0F80 & ((uint16_t) rtrans->rx_data[2] << 7)) | (rtrans->rx_data[3] >> 1);
      }
    }
};

extern AS5145 as;

