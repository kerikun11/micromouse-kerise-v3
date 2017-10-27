#pragma once

#include <WiFi.h>
#include <queue>
#include <string>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define BLE_TASK_STACK_SIZE   4096
#define BLE_TASK_PRIORITY     1

class BLETransmitter {
  public:
    BLETransmitter() {}
    void begin() {
      // Create the BLE Device
      BLEDevice::init("UART Service");

      // Create the BLE Server
      pServer = BLEDevice::createServer();

      // Create the BLE Service
      BLEService *pService = pServer->createService(SERVICE_UUID);

      // Create a BLE Characteristic
      pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
      pCharacteristic->addDescriptor(new BLE2902());
      BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);

      // Start the service
      pService->start();

      // Start advertising
      pServer->getAdvertising()->start();
      log_i("Waiting a client connection to notify...");

      queueSemaphore = xSemaphoreCreateBinary();
      xTaskCreate([](void* obj) {
        static_cast<BLETransmitter*>(obj)->task();
      }, "BLETransmitter", BLE_TASK_STACK_SIZE, this, BLE_TASK_PRIORITY, NULL);
    }
    void printf(const char* format, ...) {
      const int buf_size = 32;
      char s[buf_size];
      va_list args;
      va_start(args, format);
      vsnprintf(s, buf_size, format, args);
      va_end(args);
      q.push(s);
      xSemaphoreGive(queueSemaphore);
      log_d("%s", s);
    }
  private:
    BLECharacteristic *pCharacteristic;
    BLEServer *pServer;
    std::queue<std::string> q;
    SemaphoreHandle_t queueSemaphore;

    void send(const char* str) {
      if (pServer->getConnectedCount()) {
        pCharacteristic->setValue((uint8_t*)str, strlen(str));
        pCharacteristic->notify();
      }
    }

    void task() {
      while (1) {
        xSemaphoreTake(queueSemaphore, portMAX_DELAY);
        while (!q.empty()) {
          send(q.front().c_str());
          q.pop();
        }
      }
    }
};

