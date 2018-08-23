#pragma once

#include "driver/spi_master.h"
#include "esp_log.h"

#define TAG "SPI"

class SPI {
public:
  static bool busInit(spi_host_device_t spi_host, gpio_num_t pin_sclk,
                      gpio_num_t pin_miso, gpio_num_t pin_mosi, int dma_chain) {
    static spi_bus_config_t bus_cfg;
    bus_cfg.mosi_io_num = pin_mosi;
    bus_cfg.miso_io_num = pin_miso;
    bus_cfg.sclk_io_num = pin_sclk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 0;
    esp_err_t ret = spi_bus_initialize(spi_host, &bus_cfg, dma_chain);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "%s", esp_err_to_name(ret));
    return ret == ESP_OK;
  }
  //   static bool add_device(spi_host_device_t spi_host, gpio_num_t pin_cs,
  //                          uint8_t mode, uint32_t clock_speed_hz) {
  //     static spi_device_interface_config_t dev_cfg;
  //     dev_cfg.command_bits = 1;
  //     dev_cfg.address_bits = 0;
  //     dev_cfg.dummy_bits = 0;
  //     dev_cfg.mode = 1;
  //     dev_cfg.duty_cycle_pos = 0;
  //     dev_cfg.cs_ena_pretrans = 0;
  //     dev_cfg.cs_ena_posttrans = 0;
  //     dev_cfg.clock_speed_hz = 10000000;
  //     dev_cfg.spics_io_num = pin_cs;
  //     dev_cfg.flags = 0;
  //     dev_cfg.queue_size = 1;
  //     dev_cfg.pre_cb = NULL;
  //     dev_cfg.post_cb = NULL;
  //     ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &encoder_spi));
  //   }
};

#undef TAG