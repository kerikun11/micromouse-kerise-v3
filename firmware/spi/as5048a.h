#pragma once

#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_err.h"
#include "config.h"

#define AS5048A_MOSI_PIN   23
#define AS5048A_MISO_PIN   19
#define AS5048A_SCLK_PIN   18
#define AS5048A_CS_PIN     15

#define AS5048A_SPI               HSPI_HOST

#define AS5048A_TASK_PRIORITY     5
#define AS5048A_TASK_STACK_SIZE   2048

#define AS5048A_PULSES            16384

class AS5048A {
  public:
    AS5048A() {}
    void begin() {
      spi_device_interface_config_t AS5048A_dev_cfg = {0};
      AS5048A_dev_cfg.mode = 1;
      AS5048A_dev_cfg.clock_speed_hz = 10000000;
      //      AS5048A_dev_cfg.spics_io_num = AS5048A_CS_PIN;
      AS5048A_dev_cfg.spics_io_num = -1;
      AS5048A_dev_cfg.queue_size = 1;
      ESP_ERROR_CHECK(spi_bus_add_device(AS5048A_SPI, &AS5048A_dev_cfg, &AS5048A_spi));
      digitalWrite(AS5048A_CS_PIN, HIGH);
      pinMode(AS5048A_CS_PIN, OUTPUT);

      xTaskCreate([](void* obj) {
        static_cast<AS5048A*>(obj)->task();
      }, "AS5048A", AS5048A_TASK_STACK_SIZE, this, AS5048A_TASK_PRIORITY, &task_handle);
    }
    void print() {
      //      printf("L: %d\tR: %d\n", getPulses(0), getPulses(1));
      printf("0,%d,%d,%d\n", AS5048A_PULSES, getRaw(0), getRaw(1));
    }
    float position(uint8_t ch) {
      float value = ((float)pulses_ovf[ch] * AS5048A_PULSES + pulses[ch]) * MACHINE_WHEEL_DIAMETER * M_PI * MACHINE_GEAR_RATIO / AS5048A_PULSES;
      if (ch == 0)value = -value;
      return value;
    }
    int getPulses(uint8_t ch) {
      int value = pulses_ovf[ch] * AS5048A_PULSES + pulses[ch];
      if (ch == 0)value = -value;
      return value;
    }
    int getRaw(uint8_t ch) {
      int value = pulses[ch];
      if (ch == 0)value = -value;
      return value;
    }
  private:
    xTaskHandle task_handle;
    spi_device_handle_t AS5048A_spi;
    int pulses[2];
    int pulses_prev[2];
    int pulses_ovf[2];

    void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);

        uint8_t rxbuf[4];
        spi_transaction_t tx = {0};
        tx.flags |= SPI_TRANS_USE_TXDATA;
        tx.tx_data[0] = 0xFF; tx.tx_data[1] = 0xFF; tx.tx_data[2] = 0xFF; tx.tx_data[3] = 0xFF;
        tx.rx_buffer = rxbuf;
        tx.length = 32;
        digitalWrite(AS5048A_CS_PIN, LOW);
        ESP_ERROR_CHECK(spi_device_transmit(AS5048A_spi, &tx));
        digitalWrite(AS5048A_CS_PIN, HIGH);

        pulses[1] = ((uint16_t)(0x3F & (rxbuf[0])) << 8) | rxbuf[1];
        pulses[0] = ((uint16_t)(0x3F & (rxbuf[2])) << 8) | rxbuf[3];
        for (int i = 0; i < 2; i++) {
          if (pulses[i] > pulses_prev[i] + AS5048A_PULSES / 2) {
            pulses_ovf[i]--;
          } else if (pulses[i] < pulses_prev[i] - AS5048A_PULSES / 2) {
            pulses_ovf[i]++;
          }
          pulses_prev[i] = pulses[i];
        }
      }
    }
};

