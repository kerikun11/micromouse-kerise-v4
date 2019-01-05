/**
 * @file ble_cheese_timer_service.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief BLE GATT Server Cheese Timer Service
 * @version 0.1
 * @date 2018-12-12
 *
 * @copyright Copyright (c) 2018 Ryotaro Onuki
 *
 */
#pragma once

#include <BLE2902.h>
#include <BLE2904.h>
#include <BLECharacteristic.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLEUUID.h>
#include <thread.h>

#include <functional>

#include "app_log.h"
#include "ble_callback_utils.h"

class BLEKeriseServie {
public:
  static const BLEUUID ServiceUUID;
  static const BLEUUID MessageCharacteristicUUID;
  static const BLEUUID PositionCharacteristicUUID;

public:
  BLEKeriseServie(BLEServer *pServer) : pServer(pServer) {
    /* Service */
    pService = pServer->createService(ServiceUUID);
    {
      /* Message Characteristic */
      pMessageCharacteristic = pService->createCharacteristic(
          MessageCharacteristicUUID, BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY);
      /* setup event callback */
      class CheeseCharacteristicCallbacks : public BLECharacteristicCallbacks {
      public:
        CheeseCharacteristicCallbacks() {}
        virtual void onRead(BLECharacteristic *pCharacteristic) override {
          logd << "Message" << std::endl;
        }
        virtual void onWrite(BLECharacteristic *pCharacteristic) override {
          logd << "Message" << std::endl;
        }
      };
      pMessageCharacteristic->setCallbacks(new CheeseCharacteristicCallbacks());
      /* BLE CUD: Characteristic User Description (0x2901) */
      BLEDescriptor *pBLE2901 =
          new BLEDescriptor(static_cast<uint16_t>(0x2901));
      pBLE2901->setValue("Message");
      pMessageCharacteristic->addDescriptor(pBLE2901);
      /* BLE CCCD: Client Characteristic Configuration Description (0x2902) */
      BLE2902 *pBLE2902 = new BLE2902();
      class BLE2902Callback : public BLEDescriptorCallbacks {
      public:
        BLE2902Callback() {}
        virtual void onRead(BLEDescriptor *pDescriptor) override {
          logd << "Message 0x2902" << std::endl;
        }
        virtual void onWrite(BLEDescriptor *pDescriptor) override {
          logd << "Message 0x2902" << std::endl;
        }
      };
      pBLE2902->setCallbacks(new BLE2902Callback());
      pMessageCharacteristic->addDescriptor(pBLE2902);
      /* BLE CPFD: Characteristic Presentation Format Descriptor (0x2904) */
      BLE2904 *pBLE2904 = new BLE2904();
      pBLE2904->setFormat(BLE2904::FORMAT_UTF8);
      pMessageCharacteristic->addDescriptor(pBLE2904);
      /* Set Initial Value */
      pMessageCharacteristic->setValue("Inital Message");
    }
    {
      /* Position Characteristic */
      pPositionCharacteristic = pService->createCharacteristic(
          PositionCharacteristicUUID, BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY);
      /* setup event callback */
      class CheeseCharacteristicCallbacks : public BLECharacteristicCallbacks {
      public:
        CheeseCharacteristicCallbacks() {}
        virtual void onRead(BLECharacteristic *pCharacteristic) override {
          logd << "Position" << std::endl;
        }
        virtual void onWrite(BLECharacteristic *pCharacteristic) override {
          logd << "Position" << std::endl;
        }
      };
      pPositionCharacteristic->setCallbacks(
          new CheeseCharacteristicCallbacks());
      /* BLE CUD: Characteristic User Description (0x2901) */
      BLEDescriptor *pBLE2901 =
          new BLEDescriptor(static_cast<uint16_t>(0x2901));
      pBLE2901->setValue("Position");
      pPositionCharacteristic->addDescriptor(pBLE2901);
      /* BLE CCCD: Client Characteristic Configuration Description (0x2902) */
      BLE2902 *pBLE2902 = new BLE2902();
      class BLE2902Callback : public BLEDescriptorCallbacks {
      public:
        BLE2902Callback() {}
        virtual void onRead(BLEDescriptor *pDescriptor) override {
          logd << "Position 0x2902" << std::endl;
        }
        virtual void onWrite(BLEDescriptor *pDescriptor) override {
          logd << "Position 0x2902" << std::endl;
        }
      };
      pBLE2902->setCallbacks(new BLE2902Callback());
      pPositionCharacteristic->addDescriptor(pBLE2902);
      /* BLE CPFD: Characteristic Presentation Format Descriptor (0x2904) */
      BLE2904 *pBLE2904 = new BLE2904();
      pBLE2904->setFormat(BLE2904::FORMAT_UINT8);
      pPositionCharacteristic->addDescriptor(pBLE2904);
    }
    /* start service */
    pService->start();
  }

protected:
  BLEServer *pServer;
  BLEService *pService;
  BLECharacteristic *pMessageCharacteristic;
  BLECharacteristic *pPositionCharacteristic;
};
