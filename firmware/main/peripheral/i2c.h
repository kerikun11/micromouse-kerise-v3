#pragma once

#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "I2C"

class I2C {
public:
  static bool install(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
                      uint32_t clk_speed = 400000) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clk_speed;
    i2c_param_config(port, &conf);
    esp_err_t ret = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "%s", esp_err_to_name(ret));
    return ret == ESP_OK;
  }

  static bool writeReg8(i2c_port_t port, uint8_t addr7, uint8_t reg8,
                        uint8_t *data, int len,
                        TickType_t ticks_to_wait = 1 / portTICK_PERIOD_MS,
                        bool ack_en = true) {
    return writeReadReg(port, addr7, &reg8, 1, data, len, nullptr, 0,
                        ticks_to_wait, ack_en);
  }
  static bool readReg8(i2c_port_t port, uint8_t addr7, uint8_t reg8,
                       uint8_t *data, int len,
                       TickType_t ticks_to_wait = 1 / portTICK_PERIOD_MS,
                       bool ack_en = true) {
    return writeReadReg(port, addr7, &reg8, 1, nullptr, 0, data, len,
                        ticks_to_wait, ack_en);
  }
  static bool writeReg16(i2c_port_t port, uint8_t addr7, uint16_t reg16,
                         uint8_t *data, int len,
                         TickType_t ticks_to_wait = 1 / portTICK_PERIOD_MS,
                         bool ack_en = true) {
    const uint8_t reg_buf[2] = {(uint8_t)((reg16 >> 8) & 0xff),
                                (uint8_t)(reg16 & 0xff)};
    return writeReadReg(port, addr7, reg_buf, 2, data, len, nullptr, 0,
                        ticks_to_wait, ack_en);
  }
  static bool readReg16(i2c_port_t port, uint8_t addr7, uint16_t reg16,
                        uint8_t *data, int len,
                        TickType_t ticks_to_wait = 1 / portTICK_PERIOD_MS,
                        bool ack_en = true) {
    const uint8_t reg_buf[2] = {(uint8_t)((reg16 >> 8) & 0xff),
                                (uint8_t)(reg16 & 0xff)};
    return writeReadReg(port, addr7, reg_buf, 2, nullptr, 0, data, len,
                        ticks_to_wait, ack_en);
  }
  static bool writeReadReg(i2c_port_t port, uint8_t addr7,
                           const uint8_t *reg_data, int reg_len,
                           const uint8_t *tx_data, int tx_len, uint8_t *rx_data,
                           int rx_len, TickType_t ticks_to_wait, bool ack_en) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    if (reg_len > 0 || tx_len > 0)
      i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, ack_en);
    if (reg_len > 0)
      i2c_master_write(cmd, (uint8_t *)reg_data, reg_len, ack_en);
    if (tx_len > 0)
      i2c_master_write(cmd, (uint8_t *)tx_data, tx_len, ack_en);
    if (rx_len > 0) {
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_READ, ack_en);
      i2c_master_read(cmd, rx_data, rx_len, I2C_MASTER_LAST_NACK);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, ticks_to_wait);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "%s", esp_err_to_name(ret));
    return ret == ESP_OK;
  }
  //   static bool writeReg(i2c_port_t port, uint8_t addr7, uint8_t reg,
  //                        const uint8_t *data, int len,
  //                        TickType_t ticks_to_wait = portMAX_DELAY) {
  //     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //     i2c_master_start(cmd);
  //     i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, true);
  //     i2c_master_write_byte(cmd, reg, true);
  //     i2c_master_write(cmd, (uint8_t *)data, len, true);
  //     i2c_master_stop(cmd);
  //     esp_err_t ret = i2c_master_cmd_begin(port, cmd, ticks_to_wait);
  //     i2c_cmd_link_delete(cmd);
  //     return ret == ESP_OK;
  //   }
  //   static bool writeReg(i2c_port_t port, uint8_t addr7, uint8_t reg,
  //                        const uint8_t *data, int len,
  //                        TickType_t ticks_to_wait = portMAX_DELAY) {
  //     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //     i2c_master_start(cmd);
  //     i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, true);
  //     i2c_master_write_byte(cmd, reg, true);
  //     i2c_master_write(cmd, (uint8_t *)data, len, true);
  //     i2c_master_stop(cmd);
  //     esp_err_t ret = i2c_master_cmd_begin(port, cmd, ticks_to_wait);
  //     i2c_cmd_link_delete(cmd);
  //     return ret == ESP_OK;
  //   }
  //   static bool readReg16(i2c_port_t port, uint8_t addr7, uint16_t reg16,
  //                         uint8_t *data, int len,
  //                         TickType_t ticks_to_wait = portMAX_DELAY) {
  //     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //     i2c_master_start(cmd);
  //     i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, true);
  //     i2c_master_write_byte(cmd, (reg16 >> 8) & 0xff, true);
  //     i2c_master_write_byte(cmd, reg16 & 0xff, true);
  //     i2c_master_start(cmd);
  //     i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_READ, true);
  //     i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
  //     i2c_master_stop(cmd);
  //     esp_err_t ret = i2c_master_cmd_begin(port, cmd, ticks_to_wait);
  //     i2c_cmd_link_delete(cmd);
  //     return ret == ESP_OK;
  //   }
  //   static bool readReg16(i2c_port_t port, uint8_t addr7, uint16_t reg16,
  //                         uint8_t *data, int len,
  //                         TickType_t ticks_to_wait = portMAX_DELAY) {
  //     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //     i2c_master_start(cmd);
  //     i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, true);
  //     i2c_master_write_byte(cmd, (reg16 >> 8) & 0xff, true);
  //     i2c_master_write_byte(cmd, reg16 & 0xff, true);
  //     i2c_master_start(cmd);
  //     i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_READ, true);
  //     i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
  //     i2c_master_stop(cmd);
  //     esp_err_t ret = i2c_master_cmd_begin(port, cmd, ticks_to_wait);
  //     i2c_cmd_link_delete(cmd);
  //     return ret == ESP_OK;
  //   }
};

#undef TAG