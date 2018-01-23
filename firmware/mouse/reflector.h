#pragma once

#include <Arduino.h>
#include <array>
#include "TaskBase.h"

#define REFLECTOR_CH_SIZE         4

class Reflector : TaskBase {
  public:
    Reflector(std::array<int8_t, REFLECTOR_CH_SIZE> tx_pins, std::array<int8_t, REFLECTOR_CH_SIZE> rx_pins): tx_pins(tx_pins), rx_pins(rx_pins) {}
    bool begin() {
      for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) {
        value[i] = 0;
        pinMode(tx_pins[i], OUTPUT);
      }
      return createTask("Reflector", 10, 4096, 1);
    }
    int16_t side(uint8_t isRight) const {
      if (isRight == 0) return read(0);
      else return read(3);;
    }
    int16_t front(uint8_t isRight) const {
      if (isRight == 0) return read(1);
      else return read(2);
    }
    int16_t read(const int8_t ch) const {
      if (ch < 0 || ch >= REFLECTOR_CH_SIZE) {
        log_e("you refered an invalid channel!");
        return 0;
      }
      return value[ch];
    }
    void csv() const {
      printf("0,1500");
      for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) printf(",%d", value[i]);
      printf("\n");
    }
    void print() const {
      printf("Reflector: ");
      for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) printf("\t%04d", value[i]);
      printf("\n");
    }
  private:
    static const int ave_num = 2;
    const std::array<int8_t, REFLECTOR_CH_SIZE> tx_pins;
    const std::array<int8_t, REFLECTOR_CH_SIZE> rx_pins;
    int16_t value_buffer[ave_num][REFLECTOR_CH_SIZE];
    int16_t value[REFLECTOR_CH_SIZE];

    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS); //< 同期
        // Buffer shift
        for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) {
          for (int j = ave_num - 1; j > 0; j--) {
            value_buffer[j][i] = value_buffer[j - 1][i];
          }
        }
        // Sampling
        for (int i = 0; i < REFLECTOR_CH_SIZE; i++) {
          uint16_t offset = analogRead(rx_pins[i]); //< オフセットをサンプル
          digitalWrite(tx_pins[i], HIGH);           //< 放電開始
          delayMicroseconds(10);                    //< 最大振幅になるまでの待ち時間
          uint16_t raw = analogRead(rx_pins[i]);    //< サンプリング
          digitalWrite(tx_pins[i], LOW);            //< 充電開始
          int temp = (int)raw - offset;           //< オフセットとの差をとる
          if (temp < 1) temp = 1;                   //< 0以下にならないように1で飽和
          value_buffer[0][i] = temp;                //< 保存
          delayMicroseconds(100);                   //< 放電時間
        }
        // LPF
        for (int8_t i = 0; i < REFLECTOR_CH_SIZE; i++) {
          int sum = 0;
          for (int j = 0; j < ave_num; j++) {
            sum += value_buffer[j][i];
          }
          value[i] = sum / ave_num;
        }
      }
    }
};

