#pragma once

#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_err.h"

#define AXIS_TASK_PRIORITY    5
#define AXIS_TASK_STACK_SIZE  2048
#define AXIS_UPDATE_PERIOD_US 1000

#define ICM20602_ACCEL_FACTOR 2048.0f
#define ICM20602_GYRO_FACTOR  16.4f

class Axis {
  public:
    Axis() {}
    void begin(bool spi_initializing) {
      if (spi_initializing) {
        // ESP-IDF SPI bus initialization
        spi_bus_config_t bus_cfg = {0};
        bus_cfg.mosi_io_num = ICM20602_MOSI_PIN;
        bus_cfg.miso_io_num = ICM20602_MISO_PIN;
        bus_cfg.sclk_io_num = ICM20602_SCLK_PIN;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.max_transfer_sz = 0; // defaults to 4094 if 0
        ESP_ERROR_CHECK(spi_bus_initialize(ICM20602_SPI_HOST, &bus_cfg, ICM20602_SPI_DMA_CHAIN));
      }
      // ESP-IDF SPI device initialization
      spi_device_interface_config_t device_cfg = {0};
      device_cfg.address_bits = 8;
      device_cfg.mode = 3;
      device_cfg.clock_speed_hz = 10000000;
      device_cfg.spics_io_num = ICM20602_CS_PIN;
      device_cfg.queue_size = 1;
      ESP_ERROR_CHECK(spi_bus_add_device(ICM20602_SPI_HOST, &device_cfg, &spi_handle));
      // calibration semaphore
      calibration_start_semaphore = xSemaphoreCreateBinary();
      calibration_end_semaphore = xSemaphoreCreateBinary();
      // sampling task execution
      xTaskCreate([](void* obj) {
        static_cast<Axis*>(obj)->task();
      }, "Axis", AXIS_TASK_STACK_SIZE, this, AXIS_TASK_PRIORITY, &task_handle);
    }
    struct MotionParameter {
      float x, y, z;
      MotionParameter(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
      inline MotionParameter operator+(const MotionParameter& obj) const {
        return MotionParameter(x + obj.x, y + obj.y, z + obj.z);
      }
      inline MotionParameter operator*(const float mul) const {
        return MotionParameter(x * mul, y * mul, z * mul);
      }
      inline MotionParameter operator/(const float div) const {
        return MotionParameter(x / div, y / div, z / div);
      }
      inline const MotionParameter& operator+=(const MotionParameter& obj) {
        x += obj.x; y += obj.y; z += obj.z; return *this;
      }
      inline const MotionParameter& operator/=(const float& div) {
        x /= div; y /= div; z /= div; return *this;
      }
    };
    void print() {
      log_d("angle: %f\t%f\t%f", angle.x * 180 / PI, angle.y * 180 / PI, angle.z * 180 / PI);
      log_d("accel: %f\t%f\t%f", accel.x, accel.y, accel.z);
    }
    void calibration(bool waitForEnd = true) {
      xSemaphoreTake(calibration_end_semaphore, 0);
      xSemaphoreGive(calibration_start_semaphore);
      if (waitForEnd) calibrationWait();
    }
    void calibrationWait() {
      xSemaphoreTake(calibration_end_semaphore, portMAX_DELAY);
    }

  public:
    MotionParameter accel, velocity, gyro, angle;

  private:
    spi_device_handle_t spi_handle;
    xTaskHandle task_handle;
    SemaphoreHandle_t calibration_start_semaphore;
    SemaphoreHandle_t calibration_end_semaphore;

    MotionParameter accel_prev, gyro_prev;
    MotionParameter accel_offset, gyro_offset;

    void task() {
      portTickType xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        update();

        // calculation of angle and velocity from motion sensor
        velocity += (accel + accel_prev) / 2 * AXIS_UPDATE_PERIOD_US / 1000000;
        accel_prev = accel;
        angle += (gyro + gyro_prev) / 2 * AXIS_UPDATE_PERIOD_US / 1000000;
        gyro_prev = gyro;

        if (xSemaphoreTake(calibration_start_semaphore, 0) == pdTRUE) {
          reset();
          for (int i = 0; i < 4 ; i++) {
            MotionParameter accel_sum, gyro_sum;
            const int ave_count = 500;
            for (int i = 0; i < ave_count; i++) {
              vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
              update();
              accel_sum += accel;
              gyro_sum += gyro;
            }
            accel_offset += accel_sum / ave_count;
            gyro_offset += gyro_sum / ave_count;
          }
          velocity = MotionParameter();
          angle = MotionParameter();
          //          log_d("gyro_offset: %f\t%f\t%f", gyro_offset.x, gyro_offset.y, gyro_offset.z);
          //          log_d("accel_offset: %f\t%f\t%f", accel_offset.x, accel_offset.y, accel_offset.z);
          xSemaphoreGive(calibration_end_semaphore);
        }
      }
    }
    void reset() {
      writeReg(0x6b, 0x81); //< power management 1
      delay(100);
      writeReg(0x6b, 0x01); //< power management 1
      writeReg(0x1b, 0x18); //< gyro range
      writeReg(0x1c, 0x18); //< accel range
      writeReg(0x1A, 0x06); //< DLPF_CFG
      delay(100);
      if (readReg(117) != 0x12) {
        log_e("whoami failed:(");
      }
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
      readReg(0x3b, rx, 14);
      bond.h = rx[0]; bond.l = rx[1];
      accel.x = bond.i / ICM20602_ACCEL_FACTOR * 1000 * 9.80665 - accel_offset.x;
      bond.h = rx[2]; bond.l = rx[3];
      accel.y = bond.i / ICM20602_ACCEL_FACTOR * 1000 * 9.80665 - accel_offset.y;
      bond.h = rx[4]; bond.l = rx[5];
      accel.z = bond.i / ICM20602_ACCEL_FACTOR * 1000 * 9.80665 - accel_offset.z;

      bond.h = rx[8]; bond.l = rx[9];
      gyro.x = bond.i / ICM20602_GYRO_FACTOR * PI / 180 - gyro_offset.x;
      bond.h = rx[10]; bond.l = rx[11];
      gyro.y = bond.i / ICM20602_GYRO_FACTOR * PI / 180 - gyro_offset.y;
      bond.h = rx[12]; bond.l = rx[13];
      gyro.z = bond.i / ICM20602_GYRO_FACTOR * PI / 180 - gyro_offset.z;
    }
    void writeReg(uint8_t reg, uint8_t data) {
      static spi_transaction_t tx = {0};
      tx.flags |= SPI_TRANS_USE_TXDATA;
      tx.addr = reg;
      tx.tx_data[0] = data;
      tx.length = 8;
      ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
    }
    uint8_t readReg(const uint8_t reg) {
      static spi_transaction_t tx = {0};
      tx.flags |= SPI_TRANS_USE_RXDATA;
      tx.addr = 0x80 | reg;
      tx.length = 8;
      ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
      return tx.rx_data[0];
    }
    void readReg(const uint8_t reg, uint8_t *rx_buffer, size_t length) {
      static spi_transaction_t tx = {0};
      tx.addr = 0x80 | reg;
      tx.tx_buffer = NULL;
      tx.rx_buffer = rx_buffer;
      tx.length = 8 * length;
      ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
    }
};

extern Axis axis;

