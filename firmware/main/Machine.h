#pragma once

#include <WiFi.h>
#include <SPIFFS.h>
#include <cstdio>
#include "global.h"
#include <iostream>
#include <sstream>

class Machine
{
public:
  Machine()
  {
  }

  enum class State
  {
    INITIAL,
    SEARCH_RUN,
    FAST_RUN,
  };

  void begin()
  {
  }

  void autoRun()
  {
    ms.restore();
    switch (state)
    {
    case INITIAL:
      break;
    case SEARCH_RUN:
      break;
    case FAST_RUN:
      break;
    }
  }

private:
};