/**
 * @file ble_battery_service.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief BLE GATT Server Battery Service
 * @version 0.1
 * @date 2018-12-12
 *
 * @copyright Copyright (c) 2018 Ryotaro Onuki
 *
 */
#pragma once

#include <BLECharacteristic.h>
#include <BLEClient.h>
#include <BLEServer.h>
#include <BLEService.h>

#include <functional>

#include "app_log.h"

class BLEBatteryService {
public:
  /**
   * @brief BLE UUIDs
   */
  static const BLEUUID ServiceUUID;
  static const BLEUUID BatteryLevelCharacteristicUUID;

public:
  /**
   * @brief Construct a new BLEBatteryService object
   * @param pServer should not be nullptr
   */
  BLEBatteryService(BLEServer *pServer);
  /**
   * @brief Set the Battery Level object
   * @param level 0-100
   */
  void setBatteryLevel(uint8_t level);
  /**
   * @brief notify the battery level characteristic
   */
  void notify();

protected:
  BLEServer *pServer;
  BLEService *pBatteryService;
  BLECharacteristic *pBatteryLevelCharacteristic;
};

class BLEBatteryServiceClient {
public:
  BLEBatteryServiceClient(BLEClient *pClient) {
    /* BLE Remote Service */
    pBatteryService = pClient->getService(BLEBatteryService::ServiceUUID);
    /* BLE Remote Characteristic */
    pBatteryLevelCharacteristic = pBatteryService->getCharacteristic(
        BLEBatteryService::BatteryLevelCharacteristicUUID);
    /* setup notify callback */
    pBatteryLevelCharacteristic->registerForNotify(
        [](BLERemoteCharacteristic *pCharacteristic, uint8_t *pData,
           size_t length, bool isNotify) {
          if (length < 1) {
            logw << "no Battery Level data" << std::endl;
            return;
          }
          uint8_t level = pData[0];
          logi << "Battery Level: " << (int)level << std::endl;
        });
    BLERemoteDescriptor *pBLE2902 = pBatteryLevelCharacteristic->getDescriptor(
        static_cast<uint16_t>(0x2902));
    uint8_t val[2] = {0x01, 0x00};
    pBLE2902->writeValue(val, 2);
  }
  uint8_t readBatteryLevel() {
    return pBatteryLevelCharacteristic->readUInt8();
  }

protected:
  BLERemoteService *pBatteryService;
  BLERemoteCharacteristic *pBatteryLevelCharacteristic;
};
