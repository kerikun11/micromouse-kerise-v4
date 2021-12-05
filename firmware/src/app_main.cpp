/**
 * @file app_main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief MicroMouse KERISE
 * @date 2019-04-02
 */
#include "machine/machine.h"
#include <esp_wifi.h>

extern "C" void app_main() {
  esp_wifi_stop();
  // Serial.begin(2000000);
  Machine machine;
  machine.init();
  machine.start();
  vTaskDelay(portMAX_DELAY);
}
