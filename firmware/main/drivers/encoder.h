#pragma once

#include <driver/spi_master.h>
#include <esp_err.h>

#define ENCODER_PULSES 16384

#define ENCODER_STACK_SIZE 4096
#define ENCODER_PRIORITY 5

class Encoder {
public:
  Encoder() { sampling_end_semaphore = xSemaphoreCreateBinary(); }
  bool begin(spi_host_device_t spi_host, int8_t pin_cs) {
    // ESP-IDF SPI device initialization
    spi_device_interface_config_t dev_cfg = {0};
    dev_cfg.command_bits = 1;
    dev_cfg.address_bits = 0;
    dev_cfg.dummy_bits = 0;
    dev_cfg.mode = 1;
    dev_cfg.duty_cycle_pos = 0;
    dev_cfg.cs_ena_pretrans = 0;
    dev_cfg.cs_ena_posttrans = 0;
    dev_cfg.clock_speed_hz = 10000000;
    dev_cfg.spics_io_num = pin_cs;
    dev_cfg.flags = 0;
    dev_cfg.queue_size = 1;
    dev_cfg.pre_cb = NULL;
    dev_cfg.post_cb = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &encoder_spi));
    xTaskCreate([](void *obj) { static_cast<Encoder *>(obj)->task(); },
                "Encoder", ENCODER_STACK_SIZE, this, ENCODER_PRIORITY, NULL);
    return true;
  }
  float position(uint8_t ch) {
    float value = ((float)pulses_ovf[ch] * ENCODER_PULSES + pulses[ch]) *
                  MACHINE_WHEEL_DIAMETER * M_PI * MACHINE_GEAR_RATIO /
                  ENCODER_PULSES;
    if (ch == 1)
      value = -value;
    return value;
  }
  int getPulses(uint8_t ch) {
    int value = pulses_ovf[ch] * ENCODER_PULSES + pulses[ch];
    if (ch == 1)
      value = -value;
    return value;
  }
  int getRaw(uint8_t ch) {
    int value = pulses[ch];
    if (ch == 1)
      value = -value;
    return value;
  }
  void print() { log_d("Encoder L:\t%f\tR:\t%f\n", position(0), position(1)); }
  void csv() {
    //      printf("0,%d,%d,%d,%d\n", ENCODER_PULSES, -ENCODER_PULSES,
    //      getRaw(0), getRaw(1)); printf("0,%d,%d,%d,%d\n", ENCODER_PULSES,
    //      -ENCODER_PULSES, getPulses(0), getPulses(1));
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
    tx.tx_data[0] = 0xFF;
    tx.tx_data[1] = 0xFF;
    tx.tx_data[2] = 0xFF;
    tx.tx_data[3] = 0xFF;
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
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
      xLastWakeTime = xTaskGetTickCount();
      update();
      xSemaphoreGive(sampling_end_semaphore);
    }
  }
};
