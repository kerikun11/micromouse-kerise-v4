#pragma once

#include "supporters/logger.h"
#include "supporters/speed_controller.h"
#include "supporters/user_interface.h"
#include "supporters/wall_detector.h"

namespace supporters {

class Supporters {
public:
  WallDetector *wd;
  SpeedController *sc;
  UserInterface *ui;

private:
  hardware::Hardware *hw;

public:
  bool init() {
    ui = new UserInterface(hw);
    wd = new WallDetector(hw);
    sc = new SpeedController(model::SpeedControllerModel,
                             model::SpeedControllerGain, hw);
    return true;
  }
};

} // namespace supporters
