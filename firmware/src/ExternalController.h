#pragma once

#include <Arduino.h>

#define EXTERNAL_CONTROLLER_TASK_PRIORITY 1
#define EXTERNAL_CONTROLLER_STACK_SIZE 4096

class ExternalController : TaskBase {
public:
  ExternalController() {}
  virtual ~ExternalController() {}
  bool begin() {
    pinMode(RX, INPUT_PULLUP);
    return createTask("ExternalController", EXTERNAL_CONTROLLER_TASK_PRIORITY,
                      EXTERNAL_CONTROLLER_STACK_SIZE);
  }

private:
  char getChar() {
    while (!Serial.available())
      delay(1);
    char c = Serial.read();
    printf("%c\n", c);
    return c;
  }
  void task() override {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);
      char c = getChar();
      switch (c) {
      case 'g':
        tof.enable();
        break;
      case 's':
        tof.disable();
        break;
      case 'p':
        tof.print();
        break;
      case 'f':
        bz.play(Buzzer::CANCEL);
        mt.free();
        break;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9': {
        float duty = (c - '0');
        duty /= 10.0f;
        fan.drive(duty);
      } break;
      }
    }
  }
};
