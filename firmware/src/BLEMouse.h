#pragma once

#include <BLEDevice.h>
#include <functional>

#include "app_log.h"
#include "ble_callback_utils.h"
#include "ble_kerise_service.h"

#define BLE_STACK_SIZE 4096
#define BLE_TASK_PRIORITY 1

class BLEMouse {
public:
  static const BLEUUID ServiceUUID;

public:
  BLEMouse() {}
  bool begin() {
    /* BLE Initialization */
    BLEDevice::init("KERISE");

    /* BLE Server */
    BLEServer *pServer = BLEDevice::createServer();

    /* BLE Advertising */
    BLEAdvertising *pBLEAdvertising = pServer->getAdvertising();
    pBLEAdvertising->start();

    /* Task */
    xTaskCreate([](void *obj) { static_cast<BLEMouse *>(obj)->task(); }, "BLE",
                BLE_STACK_SIZE, this, BLE_TASK_PRIORITY, NULL);
    return true;
  }

private:
  void task() {
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    }
  }
};
