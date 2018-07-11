#pragma once

#include "global.h"
#include <SPIFFS.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>

#define MACHINE_SAVE_PATH "/spiffs/machine.bin"

class Machine {
public:
  Machine() {}

  enum State {
    INITIAL,
    SEARCH_RUN,
    FAST_RUN,
  };
  struct SaveData {
    enum State state;
  };

  void begin() {
    // SPIバスの安定化
    for (auto pin : {AS5048A_CS_PIN, ICM20602_L_CS_PIN, ICM20602_R_CS_PIN}) {
      gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
      gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLUP_ONLY);
    }
    // Boot
    std::cout << std::endl;
    std::cout << "**************** KERISE ****************" << std::endl;
    if (!bz.begin())
      bz.play(Buzzer::ERROR);
    if (!led.begin(LED_SDA_PIN, LED_SCL_PIN))
      bz.play(Buzzer::ERROR);
    ui.batteryCheck();
    bz.play(Buzzer::BOOT);

    if (!SPIFFS.begin(true))
      bz.play(Buzzer::ERROR);
    if (!imu.begin(ICM20602_SPI_HOST, ICM20602_CS_PINS, true, ICM20602_SCLK_PIN,
                   ICM20602_MISO_PIN, ICM20602_MOSI_PIN,
                   ICM20602_SPI_DMA_CHAIN))
      bz.play(Buzzer::ERROR);
    if (!enc.begin(AS5048A_SPI_HOST, AS5048A_CS_PIN, false, AS5048A_SCLK_PIN,
                   AS5048A_MISO_PIN, AS5048A_MOSI_PIN, AS5048A_SPI_DMA_CHAIN))
      bz.play(Buzzer::ERROR);
    if (!ref.begin())
      bz.play(Buzzer::ERROR);
    if (!tof.begin())
      bz.play(Buzzer::ERROR);
    if (!wd.begin())
      bz.play(Buzzer::ERROR);
    em.begin();
  }

  void autoRun() {
    restore();
    switch (saveData.state) {
    case INITIAL:
      break;
    case SEARCH_RUN:
      ms.restore();
      break;
    case FAST_RUN:
      ms.restore();
      break;
    }
  }

private:
  State state;
  SaveData saveData;

  bool restore() {
    auto f = std::ifstream(MACHINE_SAVE_PATH);
    if (f.fail()) {
      log_e("failed to open %s", MACHINE_SAVE_PATH);
      return false;
    }
    f.read((char *)&saveData, sizeof(SaveData));
    return true;
  }
  bool backup() {
    auto f = std::ifstream(MACHINE_SAVE_PATH);
    if (f.fail()) {
      log_e("failed to open %s", MACHINE_SAVE_PATH);
      return false;
    }
    f.read((char *)&saveData, sizeof(SaveData));
    return true;
  }
};