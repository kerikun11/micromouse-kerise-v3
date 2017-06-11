#pragma once

#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_err.h"
#include "TaskBase.h"
#include "config.h"

#define MPU6500_MOSI_PIN  27
#define MPU6500_MISO_PIN  35
#define MPU6500_SCLK_PIN  26
#define MPU6500_CS_PIN    25

#define MPU6500_SPI               VSPI_HOST
#define MPU6500_DMA_CHAIN         2

#define MPU6500_UPDATE_PERIOD_US  1000

#define MPU6500_ACCEL_FACTOR      2048.0f
#define MPU6500_GYRO_FACTOR       16.3835f

#define MPU6500_TASK_PRIORITY   5
#define MPU6500_TASK_STACK_SIZE 4096

class MPU6500: public TaskBase {
  public:
    //    MPU6500(): TaskBase("MPU6500", MPU6500_TASK_PRIORITY, MPU6500_TASK_STACK_SIZE) {}
    MPU6500(): TaskBase("MPU6500", MPU6500_TASK_PRIORITY, MPU6500_TASK_STACK_SIZE), spi(VSPI) {}
    virtual ~MPU6500() {}
    void init() {
      //      static spi_bus_config_t bus_cfg = {0};
      //      bus_cfg.mosi_io_num = MPU6500_MOSI_PIN;
      //      bus_cfg.miso_io_num = MPU6500_MISO_PIN;
      //      bus_cfg.sclk_io_num = MPU6500_SCLK_PIN;
      //      bus_cfg.quadwp_io_num = -1;
      //      bus_cfg.quadhd_io_num = -1;
      //      ESP_ERROR_CHECK(spi_bus_initialize(MPU6500_SPI, &bus_cfg, MPU6500_DMA_CHAIN));
      //      static spi_device_interface_config_t device_cfg = {0};
      //      device_cfg.address_bits = 8;
      //      device_cfg.mode = 0;
      //      device_cfg.clock_speed_hz = 10000000;
      //      device_cfg.spics_io_num = MPU6500_CS_PIN;
      //      device_cfg.queue_size = 1;
      //      ESP_ERROR_CHECK(spi_bus_add_device(MPU6500_SPI, &device_cfg, &spi_handle));

      spi.begin(MPU6500_SCLK_PIN, MPU6500_MISO_PIN, MPU6500_MOSI_PIN, MPU6500_CS_PIN);
      pinMode(MPU6500_CS_PIN, OUTPUT);
      digitalWrite(MPU6500_CS_PIN, HIGH);

      create_task();
    }
    struct Parameter {
      float x, y, z;
      Parameter(float x = 0, float y = 0, float z = 0) :
        x(x), y(y), z(z) {
      }
      inline Parameter operator+(const Parameter& obj) const {
        return Parameter(x + obj.x, y + obj.y, z + obj.z);
      }
      inline Parameter operator*(const float mul) const {
        return Parameter(x * mul, y * mul, z * mul);
      }
      inline Parameter operator/(const float div) const {
        return Parameter(x / div, y / div, z / div);
      }
      inline const Parameter& operator+=(const Parameter& obj) {
        x += obj.x; y += obj.y; z += obj.z; return *this;
      }
      inline const Parameter& operator/=(const float& div) {
        x /= div; y /= div; z /= div; return *this;
      }
    };
    Parameter accel, velocity, gyro, angle;
    void calibration(bool waitUntilTheEnd = true) {
      delete_task();
      writeReg(0x19, 0x07);
      writeReg(0x1b, 0x18);
      writeReg(0x1c, 0x18);
      delay(100);
      create_task();
      delay(100);
      calibration_flag = true;
      if (waitUntilTheEnd) {
        calibrationWait();
      }
    }
    void calibrationWait() {
      while (calibration_flag) {
        vTaskDelay(1 / portTICK_RATE_MS);
      }
    }
    void print() {
      printf("angle:\t(%.3f,\t%.3f,\t%.3f)\n", angle.x * 180 / PI, angle.y * 180 / PI, angle.z * 180 / PI);
    }
  private:
    //    spi_device_handle_t spi_handle;
    SPIClass spi;
    Parameter accel_offset;
    Parameter gyro_offset;
    Parameter accel_prev;
    Parameter gyro_prev;
    bool calibration_flag;

    virtual void task() {
      writeReg(0x19, 0x07);
      writeReg(0x1b, 0x18);
      writeReg(0x1c, 0x18);
      delay(100);

      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        update();
        velocity += (accel + accel_prev) / 2 * MPU6500_UPDATE_PERIOD_US / 1000000;
        accel_prev = accel;
        angle += (gyro + gyro_prev) / 2 * MPU6500_UPDATE_PERIOD_US / 1000000;
        gyro_prev = gyro;
      }
    }
    void writeReg(uint8_t reg, uint8_t data) {
      //      static spi_transaction_t tx = {0};
      //      tx.flags |= SPI_TRANS_USE_TXDATA;
      //      tx.address = (uint32_t)reg << 24;
      //      tx.tx_data[0] = data;
      //      tx.length = 8;
      //      ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
      spi.beginTransaction(SPISettings(10000000, SPI_MSBFIRST, SPI_MODE0));
      digitalWrite(MPU6500_CS_PIN, LOW);
      spi.transfer(reg);
      spi.transfer(data);
      spi.endTransaction();
      digitalWrite(MPU6500_CS_PIN, HIGH);
    }
    void readReg(uint8_t reg, uint8_t *rx_buffer, int length) {
      //      static spi_transaction_t tx = {0};
      //      tx.address = (0x80 | reg) << 24;
      //      tx.rx_buffer = rx_buffer;
      //      tx.length = 8 * length;
      //      ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
      spi.beginTransaction(SPISettings(10000000, SPI_MSBFIRST, SPI_MODE0));
      digitalWrite(MPU6500_CS_PIN, LOW);
      spi.transfer(0x80 | reg);
      for (int i = 0; i < length; i++)
        rx_buffer[i] = spi.transfer(0x00);
      spi.endTransaction();
      digitalWrite(MPU6500_CS_PIN, HIGH);
    }
    void update() {
      union {
        int16_t i;
        struct {
          uint8_t l : 8;
          uint8_t h : 8;
        };
      } bond;
      uint8_t rx[14];
      readReg(0x3B, rx, 14);
      bond.h = rx[0]; bond.l = rx[1];
      accel.x = bond.i / MPU6500_ACCEL_FACTOR * 1000 * 9.80665 - accel_offset.x;
      bond.h = rx[2]; bond.l = rx[3];
      accel.y = bond.i / MPU6500_ACCEL_FACTOR * 1000 * 9.80665 - accel_offset.y;
      bond.h = rx[4]; bond.l = rx[5];
      accel.z = bond.i / MPU6500_ACCEL_FACTOR * 1000 * 9.80665 - accel_offset.z;

      bond.h = rx[8]; bond.l = rx[9];
      gyro.x = bond.i / MPU6500_GYRO_FACTOR * PI / 180 - gyro_offset.x;
      bond.h = rx[10]; bond.l = rx[11];
      gyro.y = bond.i / MPU6500_GYRO_FACTOR * PI / 180 - gyro_offset.y;
      bond.h = rx[12]; bond.l = rx[13];
      gyro.z = bond.i / MPU6500_GYRO_FACTOR * PI / 180 - gyro_offset.z;

      if (calibration_flag) {
        static Parameter accel_sum, gyro_sum;
        accel_sum += accel;
        gyro_sum += gyro;
        static int calibration_counter;
        const int ave_count = 1000;
        if (++calibration_counter >= ave_count) {
          accel_offset += accel_sum / ave_count;
          gyro_offset += gyro_sum / ave_count;
          calibration_counter = 0;
          calibration_flag = false;
          accel_sum = Parameter();
          gyro_sum = Parameter();
          velocity = Parameter();
          angle = Parameter();
        }
      }
    }
};

extern MPU6500 mpu;

