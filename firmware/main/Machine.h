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
	  START_CELL,
	  BW_START_AND_GOAL,
	  BACKING_TO_START,
  };
  struct SaveData {
    enum State state;
	int track = 0;
  };

  void begin() {
  }

  void autoRun() {
    restore();
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