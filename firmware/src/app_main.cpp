/**
 * @file app_main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief MicroMouse KERISE firmware
 * @date 2019-04-02
 */
#include "app_log.h"
#include "machine/machine.h"

void kerise_main() {
  /* pull-down motor pin */
  gpio_config_t config = {
      .pin_bit_mask = ((uint64_t)1 << MOTOR_L_CTRL1_PIN) |
                      ((uint64_t)1 << MOTOR_L_CTRL2_PIN) |
                      ((uint64_t)1 << MOTOR_R_CTRL1_PIN) |
                      ((uint64_t)1 << MOTOR_R_CTRL2_PIN) |
                      ((uint64_t)1 << FAN_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&config));
  auto machine = new machine::Machine();
  machine->init();
  vTaskDelay(pdMS_TO_TICKS(1000));
  LOGI("Free Heap: %u [Bytes]", esp_get_free_heap_size());
  vTaskDelay(portMAX_DELAY);
}

void devkit_main() {
  LOGI("This is ESP32 DevKit");
#if 0
  auto *bz = hardware::Buzzer::get_instance();
  bz->init(BUZZER_PIN, BUZZER_LEDC_CHANNEL, BUZZER_LEDC_TIMER);
  bz->play(hardware::Buzzer::BOOT);
#endif
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
