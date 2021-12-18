/**
 * @file app_main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief MicroMouse KERISE
 * @date 2019-04-02
 */
#include "machine/machine.h"

extern "C" void app_main() {
  auto machine = new machine::Machine;
  machine->init();
  machine->start();
  vTaskDelay(portMAX_DELAY);
}
