/**
 * @file app_main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief MicroMouse KERISE
 * @date 2019-04-02
 */
#include "machine/machine.h"

#include <HardwareSerial.h>
#include <esp_wifi.h>

void setup() {
  esp_wifi_stop();
  Serial.begin(2000000);
  Machine machine;
  machine.init();
  vTaskDelay(portMAX_DELAY);
}

void loop() { vTaskDelay(portMAX_DELAY); }
