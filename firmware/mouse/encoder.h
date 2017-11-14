#pragma once

#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_err.h"

#define ENCODER_TASK_PRIORITY     5
#define ENCODER_TASK_STACK_SIZE   2048

#define ENCODER_PULSES            16384

class Encoder {
  public:
    Encoder() {}
    void begin(bool spi_initializing) {
      if (spi_initializing) {
        // ESP-IDF SPI bus initialization
        spi_bus_config_t bus_cfg = {0};
        bus_cfg.mosi_io_num = AS5048A_MOSI_PIN;
        bus_cfg.miso_io_num = AS5048A_MISO_PIN;
        bus_cfg.sclk_io_num = AS5048A_SCLK_PIN;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.max_transfer_sz = 0; // defaults to 4094 if 0
        ESP_ERROR_CHECK(spi_bus_initialize(AS5048A_SPI_HOST, &bus_cfg, AS5048A_SPI_DMA_CHAIN));
      }
      // ESP-IDF SPI device initialization
      spi_device_interface_config_t encoder_dev_cfg = {0};
      encoder_dev_cfg.mode = 1;
      encoder_dev_cfg.clock_speed_hz = 1000000;
      //      encoder_dev_cfg.spics_io_num = AS5048A_CS_PIN;
      encoder_dev_cfg.spics_io_num = -1;
      encoder_dev_cfg.queue_size = 1;
      encoder_dev_cfg.pre_cb = [](spi_transaction_t* tx) {
        digitalWrite(AS5048A_CS_PIN, LOW);
      };
      encoder_dev_cfg.post_cb = [](spi_transaction_t* tx) {
        digitalWrite(AS5048A_CS_PIN, HIGH);
      };
      ESP_ERROR_CHECK(spi_bus_add_device(AS5048A_SPI_HOST, &encoder_dev_cfg, &encoder_spi));
      digitalWrite(AS5048A_CS_PIN, HIGH);
      pinMode(AS5048A_CS_PIN, OUTPUT);

      xTaskCreate([](void* obj) {
        static_cast<Encoder*>(obj)->task();
      }, "Encoder", ENCODER_TASK_STACK_SIZE, this, ENCODER_TASK_PRIORITY, &task_handle);
    }
    void print() {
      printf("L: %d\tR: %d\n", getRaw(0), getRaw(1));
    }
    float position(uint8_t ch) {
      float value = ((float)pulses_ovf[ch] * ENCODER_PULSES + pulses[ch]) * MACHINE_WHEEL_DIAMETER * M_PI * MACHINE_GEAR_RATIO / ENCODER_PULSES;
      if (ch == 1)value = -value;
      return value;
    }
    int getPulses(uint8_t ch) {
      int value = pulses_ovf[ch] * ENCODER_PULSES + pulses[ch];
      if (ch == 1)value = -value;
      return value;
    }
    int getRaw(uint8_t ch) {
      int value = pulses[ch];
      if (ch == 1)value = -value;
      return value;
    }
    void csv() {
      //      printf("0,%d,%d,%d,%d\n", ENCODER_PULSES, -ENCODER_PULSES, getRaw(0), getRaw(1));
      //      printf("0,%d,%d,%d,%d\n", ENCODER_PULSES, -ENCODER_PULSES, getPulses(0), getPulses(1));
      printf("0,%f,%f\n", position(0), position(1));
    }
  private:
    xTaskHandle task_handle;
    spi_device_handle_t encoder_spi;
    int pulses[2];
    int pulses_prev[2];
    int pulses_ovf[2];

    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);

        uint8_t rxbuf[4];
        spi_transaction_t tx = {0};
        tx.flags |= SPI_TRANS_USE_TXDATA;
        tx.tx_data[0] = 0xFF; tx.tx_data[1] = 0xFF; tx.tx_data[2] = 0xFF; tx.tx_data[3] = 0xFF;
        tx.rx_buffer = rxbuf;
        tx.length = 32;
        ESP_ERROR_CHECK(spi_device_transmit(encoder_spi, &tx));

        pulses[1] = ((uint16_t)(0x3F & (rxbuf[0])) << 8) | rxbuf[1];
        pulses[0] = ((uint16_t)(0x3F & (rxbuf[2])) << 8) | rxbuf[3];
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

