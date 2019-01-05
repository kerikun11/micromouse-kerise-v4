/**
 * @file ble_cheese_timer_service.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief BLE GATT Server Cheese Timer Service
 * @version 0.1
 * @date 2018-12-12
 *
 * @copyright Copyright (c) 2018 Ryotaro Onuki
 *
 */
#include "ble_kerise_service.h"

#include <BLE2902.h>
#include <BLE2904.h>

#include <iomanip>
#include <iostream>

#include "app_log.h"

const BLEUUID BLEKeriseServie::ServiceUUID =
    BLEUUID("e2840000-fdb1-11e8-8eb2-f2801f1b9fd1");

const BLEUUID BLEKeriseServie::MessageCharacteristicUUID =
    BLEUUID("e2840001-fdb1-11e8-8eb2-f2801f1b9fd1");

const BLEUUID BLEKeriseServie::PositionCharacteristicUUID =
    BLEUUID("e2840002-fdb1-11e8-8eb2-f2801f1b9fd1");
