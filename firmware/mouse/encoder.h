#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "TaskBase.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "config.h"

#define ENCODER_TASK_PRIORITY   5
#define ENCODER_TASK_STACK_SIZE 4096

#define ENCODER_SPI_HOST        HSPI_HOST
#define ENCODER_DMA_CHAIN       1

#define ENCODER_PULSES          4096

class Encoder: private TaskBase {
  public:
    Encoder(): TaskBase("Encoder", ENCODER_TASK_PRIORITY, ENCODER_TASK_STACK_SIZE), spi(HSPI) {}
    //    Encoder(): TaskBase("Encoder", ENCODER_TASK_PRIORITY, ENCODER_TASK_STACK_SIZE) {}
    virtual ~Encoder() {}
    void init() {
      //      static spi_bus_config_t bus_cfg = {0};
      //      bus_cfg.sclk_io_num = AS5145_SCLK_PIN;
      //      bus_cfg.miso_io_num = AS5145_MISO_PIN;
      //      bus_cfg.mosi_io_num = -1;
      //      bus_cfg.quadhd_io_num = -1;
      //      bus_cfg.quadwp_io_num = -1;
      //      ESP_ERROR_CHECK(spi_bus_initialize(ENCODER_SPI, &bus_cfg, ENCODER_DMA_CHAIN));
      //
      //      static spi_device_interface_config_t encoder_dev_cfg = {0};
      //      encoder_dev_cfg.flags = 0;
      //      encoder_dev_cfg.clock_speed_hz = 500000;
      //      encoder_dev_cfg.mode = 4;
      //      encoder_dev_cfg.spics_io_num = AS5145_CS_PIN;
      //      encoder_dev_cfg.queue_size = 1;
      //      ESP_ERROR_CHECK(spi_bus_add_device(ENCODER_SPI, &encoder_dev_cfg, &ENCODER_SPI_HOST));
      spi.begin(AS5145_SCLK_PIN, AS5145_MISO_PIN, AS5145_MOSI_PIN, AS5145_CS_PIN);
      spi.setDataMode(SPI_MODE1);
      digitalWrite(AS5145_CS_PIN, HIGH);
      pinMode(AS5145_CS_PIN, OUTPUT);
      create_task();
    }
    void print() {
      printf("L: %d\tR: %d\n", getPulses(0), getPulses(1));
    }
    float position(uint8_t ch) {
      float value = ((float)pulses_ovf[ch] * ENCODER_PULSES + pulses[ch]) * MACHINE_WHEEL_DIAMETER * M_PI * MACHINE_GEAR_RATIO / ENCODER_PULSES;
      if (ch == 0)value = -value;
      return value;
    }
    int getPulses(uint8_t ch) {
      int value = pulses_ovf[ch] * ENCODER_PULSES + pulses[ch];
      if (ch == 0)value = -value;
      return value;
    }
    int getRaw(uint8_t ch) {
      int value = pulses[ch];
      if (ch == 0)value = -value;
      return value;
    }
  private:
    //    spi_device_handle_t ENCODER_SPI_HOST;
    SPIClass spi;
    int pulses[2];
    int pulses_prev[2];
    int pulses_ovf[2];

    virtual void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);

        //        spi_transaction_t tx = {0};
        //        uint8_t txbuf[5];
        //        uint8_t rxbuf[5];
        //        tx.length = 38;
        //        tx.tx_buffer = txbuf;
        //        tx.rx_buffer = rxbuf;
        //        ESP_ERROR_CHECK(spi_device_transmit(ENCODER_SPI_HOST, &tx));

        uint8_t rxbuf[5];
        digitalWrite(AS5145_CS_PIN, LOW);
        //        spi.beginTransaction(SPISettings(1000000, SPI_MSBFIRST, SPI_MODE0));
        rxbuf[0] = spi.transfer(0xFF);
        rxbuf[1] = spi.transfer(0xFF);
        rxbuf[2] = spi.transfer(0xFF);
        rxbuf[3] = spi.transfer(0xFF);
        rxbuf[4] = spi.transfer(0xFF);
        //        spi.endTransaction();
        digitalWrite(AS5145_CS_PIN, HIGH);

        pulses[0] = ((0x1F & (uint16_t)rxbuf[2]) << 7) | (rxbuf[3] >> 1);
        pulses[1] =  ((uint16_t)rxbuf[0] << 4) | (rxbuf[1] >> 4);
        for (int i = 0; i < 2; i++) {
          if (pulses[i] > pulses_prev[i] + ENCODER_PULSES / 2) {
            pulses_ovf[i]--;
          } else if (pulses[i] < pulses_prev[i] - ENCODER_PULSES / 2) {
            pulses_ovf[i]++;
          }
          pulses_prev[i] = pulses[i];
        }
      }
    }
};

extern Encoder enc;

