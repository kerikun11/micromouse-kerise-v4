/**
 * @file ble_battery_service.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief BLE GATT Server Battery Service
 * @version 0.1
 * @date 2018-12-12
 *
 * @copyright Copyright (c) 2018 Ryotaro Onuki
 *
 */
#include "ble_battery_service.h"

#include <BLE2902.h>
#include <BLE2904.h>
#include <BLEDevice.h>

#include <iomanip>
#include <iostream>

#include "app_log.h"

const BLEUUID BLEBatteryService::ServiceUUID =
    BLEUUID(static_cast<uint16_t>(0x180f));

const BLEUUID BLEBatteryService::BatteryLevelCharacteristicUUID =
    BLEUUID(static_cast<uint16_t>(0x2a19));

BLEBatteryService::BLEBatteryService(BLEServer *pServer) : pServer(pServer) {
  /* Battery Service */
  pBatteryService = pServer->createService(static_cast<uint16_t>(0x180f));
  /* Battery Level Characteristic */
  pBatteryLevelCharacteristic = pBatteryService->createCharacteristic(
      static_cast<uint16_t>(0x2a19),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  /* setup event callback */
  class BatteryLevelCharacteristicCallbacks
      : public BLECharacteristicCallbacks {
  public:
    BatteryLevelCharacteristicCallbacks() {}
    virtual void onRead(BLECharacteristic *pCharacteristic) override {
      logd << "BatteryLevel" << std::endl;
    }
    virtual void onWrite(BLECharacteristic *pCharacteristic) override {
      logd << "BatteryLevel" << std::endl;
    }
  };
  pBatteryLevelCharacteristic->setCallbacks(
      new BatteryLevelCharacteristicCallbacks());
  /* BLE CCCD: Client Characteristic Configuration Description (0x2902) */
  BLE2902 *pBLE2902 = new BLE2902();
  class BLE2902Callback : public BLEDescriptorCallbacks {
  public:
    BLE2902Callback() {}
    virtual void onRead(BLEDescriptor *pDescriptor) override {
      logd << "BatteryLevel 0x2902" << std::endl;
    }
    virtual void onWrite(BLEDescriptor *pDescriptor) override {
      logd << "BatteryLevel 0x2902" << std::endl;
    }
  };
  pBLE2902->setCallbacks(new BLE2902Callback());
  pBatteryLevelCharacteristic->addDescriptor(pBLE2902);
  /* BLE CPFD: Characteristic Presentation Format Descriptor (0x2904) */
  BLE2904 *pBLE2904 = new BLE2904();
  pBLE2904->setFormat(BLE2904::FORMAT_UINT8);
  pBatteryLevelCharacteristic->addDescriptor(pBLE2904);
  /* start service */
  pBatteryService->start();
}

void BLEBatteryService::setBatteryLevel(uint8_t level) {
  pBatteryLevelCharacteristic->setValue(&level, 1);
}

void BLEBatteryService::notify() { pBatteryLevelCharacteristic->notify(); }