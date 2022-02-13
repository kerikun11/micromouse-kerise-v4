/**
 * @file app_main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief MicroMouse KERISE firmware
 * @date 2019-04-02
 */
#include "app_log.h"
#include "machine/machine.h"

void kerise_main() {
  auto machine = new machine::Machine;
  machine->init();
  machine->start();
  vTaskDelay(pdMS_TO_TICKS(1000));
  LOGI("Free Heap: %u [Bytes]", esp_get_free_heap_size());
  vTaskDelay(portMAX_DELAY);
}

void devkit_main() {
  LOGI("This is ESP32 DevKit");
  auto *bz = hardware::Buzzer::get_instance();
  bz->init(BUZZER_PIN, BUZZER_LEDC_CHANNEL, BUZZER_LEDC_TIMER);
  bz->play(hardware::Buzzer::BOOT);
  vTaskDelay(portMAX_DELAY);
}

extern "C" void app_main() {
  /* Check ID */
  uint64_t mac = peripheral::ESP::get_mac();
  switch (mac) {
  case 0xD4E3'CAA4'AE30: //< DevKit
    return devkit_main();
  case 0x080C'401D'A0D8: //< KERISE v4
  case 0x807F'631D'A0D8: //< KERISE v4 Copy
  case 0xD866'5A1D'A0D8: //< KERISE v5
    return kerise_main();
  default:
    LOGW("unknown ESP32 MAC: 0x%012llX", mac);
    return devkit_main();
  }
}
