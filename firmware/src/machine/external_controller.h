#pragma once

#include <HardwareSerial.h>
#include <pins_arduino.h>

class ExternalController {
public:
  ExternalController() {}
  bool init() {
    pinMode(RX, INPUT_PULLUP);
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "EC", 4096, this, 1, NULL);
    return true;
  }

private:
  char getChar() {
    while (!Serial.available())
      delay(1);
    char c = Serial.read();
    printf("%c\n", c);
    return c;
  }
  void task() {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
      char c = getChar();
      switch (c) {
      case 'c':
        imu.calibration();
        bz.play(Buzzer::CALIBRATION);
        imu.angle = 0;
        break;
      case 'g':
        // tof.enable();
        break;
      case 's':
        // tof.disable();
        break;
      case 'p':
        break;
      case 'f':
        bz.play(Buzzer::CANCEL);
        mt.free();
        fan.drive(0);
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
        duty /= 10;
        mt.drive(duty, duty);
        // fan.drive(duty);
      } break;
      }
    }
  }
};
