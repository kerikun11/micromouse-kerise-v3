#pragma once

#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_err.h"
#include "task_base.h"

#define MPU6500_MOSI_PIN  27
#define MPU6500_MISO_PIN  35
#define MPU6500_SCLK_PIN  26
#define MPU6500_CS_PIN    25

#define MPU6500_DMA_CHAIN 1

class MPU6500: public TaskBase {
  public:
    MPU6500(): TaskBase("MPU6500 Task", 1, 512) {
      spi_bus_config_t bus_cfg = {0};
      bus_cfg.mosi_io_num = MPU6500_MOSI_PIN;
      bus_cfg.miso_io_num = MPU6500_MISO_PIN;
      bus_cfg.sclk_io_num = MPU6500_SCLK_PIN;
      bus_cfg.quadwp_io_num = -1;
      bus_cfg.quadhd_io_num = -1;
      ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_cfg, MPU6500_DMA_CHAIN));
      spi_device_interface_config_t device_cfg = {0};
      device_cfg.address_bits = 8;
      device_cfg.mode = 0;
      device_cfg.clock_speed_hz = 1000000;
      //      device_cfg.spics_io_num = MPU6500_CS_PIN;
      device_cfg.spics_io_num = -1;
      device_cfg.queue_size = 1;
      ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &device_cfg, &spi_handle));
    }
    virtual ~MPU6500() {
    }
    void init() {
      digitalWrite(MPU6500_CS_PIN, HIGH);
      pinMode(MPU6500_CS_PIN, OUTPUT);
      //      writeReg(0x6b, 0x80);
      //      delay(100);
      //      writeReg(0x19, 0x07);
    }
    void print() {
      uint8_t data[14];
      readReg(0x3B, data, 14);
      for (int i = 0; i < 14; i++) {
        printf("%d\t", data[i]);
      }
      printf("\n");
    }
    void writeReg(uint8_t reg, uint8_t data) {
      spi_transaction_t tx = {0};
      tx.flags |= SPI_TRANS_USE_TXDATA;
      tx.flags |= SPI_TRANS_USE_RXDATA;
      tx.address = 0x00 | reg;
      tx.tx_data[0] = data;
      tx.length = 8;
      digitalWrite(MPU6500_CS_PIN, LOW);
      ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
      digitalWrite(MPU6500_CS_PIN, HIGH);
    }
    void readReg(uint8_t reg, uint8_t *data, int length) {
      spi_transaction_t tx = {0};
      tx.address = 0x80 | reg;
      tx.length = 8 * length;
      tx.rx_buffer = data;
      digitalWrite(MPU6500_CS_PIN, LOW);
      ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
      digitalWrite(MPU6500_CS_PIN, HIGH);
    }
  private:
    spi_device_handle_t spi_handle;
    virtual void task() {
    }
};

extern MPU6500 mpu;

