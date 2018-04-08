#pragma once

#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_err.h"

#define ENCODER_PULSES      16384

#define ENCODER_STACK_SIZE  4096
#define ENCODER_PRIORITY    5

class Encoder {
  public:
    Encoder() {
      sampling_end_semaphore = xSemaphoreCreateBinary();
    }
    bool begin(spi_host_device_t spi_host,
               int8_t pin_cs,
               bool spi_bus_initializing = false,
               int8_t pin_sclk = -1, int8_t pin_miso = -1, int8_t pin_mosi = -1,
               int dma_chain = 0) {
      if (spi_bus_initializing) {
        // ESP-IDF SPI bus initialization
        spi_bus_config_t bus_cfg = {0};
        bus_cfg.mosi_io_num = pin_mosi; ///< GPIO pin for Master Out Slave In (=spi_d) signal, or -1 if not used.
        bus_cfg.miso_io_num = pin_miso; ///< GPIO pin for Master In Slave Out (=spi_q) signal, or -1 if not used.
        bus_cfg.sclk_io_num = pin_sclk; ///< GPIO pin for Spi CLocK signal, or -1 if not used.
        bus_cfg.quadwp_io_num = -1;     ///< GPIO pin for WP (Write Protect) signal which is used as D2 in 4-bit communication modes, or -1 if not used.
        bus_cfg.quadhd_io_num = -1;     ///< GPIO pin for HD (HolD) signal which is used as D3 in 4-bit communication modes, or -1 if not used.
        bus_cfg.max_transfer_sz = 0;    ///< Maximum transfer size, in bytes. Defaults to 4094 if 0.
        ESP_ERROR_CHECK(spi_bus_initialize(spi_host, &bus_cfg, dma_chain));
      }
      // ESP-IDF SPI device initialization
      spi_device_interface_config_t dev_cfg = {0};
      dev_cfg.command_bits = 1;         ///< Default amount of bits in command phase (0-16), used when ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored.
      dev_cfg.address_bits = 0;         ///< Default amount of bits in address phase (0-64), used when ``SPI_TRANS_VARIABLE_ADDR`` is not used, otherwise ignored.
      dev_cfg.dummy_bits = 0;           ///< Amount of dummy bits to insert between address and data phase
      dev_cfg.mode = 1;                 ///< SPI mode (0-3)
      dev_cfg.duty_cycle_pos = 0;       ///< Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
      dev_cfg.cs_ena_pretrans = 0;      ///< Amount of SPI bit-cycles the cs should be activated before the transmission (0-16). This only works on half-duplex transactions.
      dev_cfg.cs_ena_posttrans = 0;     ///< Amount of SPI bit-cycles the cs should stay active after the transmission (0-16)
      dev_cfg.clock_speed_hz = 10000000;///< Clock speed, in Hz
      dev_cfg.spics_io_num = pin_cs;    ///< CS GPIO pin for this device, or -1 if not used
      dev_cfg.flags = 0;                ///< Bitwise OR of SPI_DEVICE_* flags
      dev_cfg.queue_size = 1;           ///< Transaction queue size. This sets how many transactions can be 'in the air' (queued using spi_device_queue_trans but not yet finished using spi_device_get_trans_result) at the same time
      dev_cfg.pre_cb = NULL;            ///< Callback to be called before a transmission is started. This callback is called within interrupt context.
      dev_cfg.post_cb = NULL;           ///< Callback to be called after a transmission has completed. This callback is called within interrupt context.
      ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &encoder_spi));
      xTaskCreate([](void* obj) {
        static_cast<Encoder*>(obj)->task();
      }, "Encoder", ENCODER_STACK_SIZE, this, ENCODER_PRIORITY, NULL);
      return true;
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
    void print() {
      log_d("Encoder L:\t%f\tR:\t%f\n", position(0), position(1));
    }
    void csv() {
      //      printf("0,%d,%d,%d,%d\n", ENCODER_PULSES, -ENCODER_PULSES, getRaw(0), getRaw(1));
      //      printf("0,%d,%d,%d,%d\n", ENCODER_PULSES, -ENCODER_PULSES, getPulses(0), getPulses(1));
      printf("0,%f,%f\n", position(0), position(1));
    }
    void samplingSemaphoreTake(portTickType xBlockTime = portMAX_DELAY) {
      xSemaphoreTake(sampling_end_semaphore, xBlockTime);
    }

  private:
    spi_device_handle_t encoder_spi;
    SemaphoreHandle_t sampling_end_semaphore;
    int pulses[2];
    int pulses_prev[2];
    int pulses_ovf[2];

    void update() {
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

    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); xLastWakeTime = xTaskGetTickCount();
        update();
        xSemaphoreGive(sampling_end_semaphore);
      }
    }
};

