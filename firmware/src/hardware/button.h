#pragma once

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Button {
public:
  static constexpr int button_sampling_time_ms = 20;
  static constexpr int button_time_press = 1;
  static constexpr int button_time_long_press_1 = 20;
  static constexpr int button_time_long_press_2 = 100;
  static constexpr int button_time_long_press_3 = 500;

  static constexpr UBaseType_t BUTTON_TASK_PRIORITY = 1;
  static constexpr uint32_t BUTTON_STACK_SIZE = 1024;

public:
  Button(const gpio_num_t pin) : pin(pin) {
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    flags = 0x00;
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "Button", BUTTON_STACK_SIZE, this, BUTTON_TASK_PRIORITY, NULL);
  }
  union {
    uint8_t flags; /**< all flags */
    struct {
      uint8_t pressed : 1;         /**< pressed */
      uint8_t long_pressed_1 : 1;  /**< long-pressed level 1 */
      uint8_t long_pressed_2 : 1;  /**< long-pressed level 2 */
      uint8_t long_pressed_3 : 1;  /**< long-pressed level 3 */
      uint8_t pressing : 1;        /**< pressing */
      uint8_t long_pressing_1 : 1; /**< long-pressing level 1 */
      uint8_t long_pressing_2 : 1; /**< long-pressing level 2 */
      uint8_t long_pressing_3 : 1; /**< long-pressing level 3 */
    };
  };

private:
  gpio_num_t pin;
  int counter = 0;

  void update() {
    if (gpio_get_level(pin) == LOW) {
      if (counter < button_time_long_press_3 + 1)
        counter++;
      if (counter == button_time_long_press_3)
        long_pressing_3 = 1;
      if (counter == button_time_long_press_2)
        long_pressing_2 = 1;
      if (counter == button_time_long_press_1)
        long_pressing_1 = 1;
      if (counter == button_time_press)
        pressing = 1;
    } else {
      if (counter >= button_time_long_press_3)
        long_pressed_3 = 1;
      else if (counter >= button_time_long_press_2)
        long_pressed_2 = 1;
      else if (counter >= button_time_long_press_1)
        long_pressed_1 = 1;
      else if (counter >= button_time_press)
        pressed = 1;
      counter = 0;
      flags &= 0x0F;
    }
  }
  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(button_sampling_time_ms));
      update();
    }
  }
};
