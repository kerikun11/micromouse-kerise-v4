/**
 * @file reflector.h
 * @brief Reflector Driver
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include <TimerSemaphore.h>
#include <array>
#include <ctrl/accumulator.h>
#include <peripheral/adc.h>

class Reflector {
public:
  static constexpr UBaseType_t Priority = 20;
  static constexpr int CH_SIZE = 4;
  static constexpr int ave_num = 8;

public:
  Reflector(const std::array<int8_t, CH_SIZE> &tx_pins,
            const std::array<int8_t, CH_SIZE> &rx_pins)
      : tx_pins(tx_pins), rx_pins(rx_pins) {}
  bool init() {
    for (int8_t i = 0; i < CH_SIZE; i++) {
      value[i] = 0;
      pinMode(tx_pins[i], OUTPUT);
      adcAttachPin(rx_pins[i]);
    }
    ts.periodic(200);
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "Reflector", configMINIMAL_STACK_SIZE * 2, this, Priority,
                NULL);
    return true;
  }
  int16_t side(uint8_t isRight) const { return read(isRight ? 1 : 0); }
  int16_t front(uint8_t isRight) const { return read(isRight ? 3 : 2); }
  int16_t read(const int8_t ch) const { return value[ch]; }
  void csv() const {
    std::cout << "0,1000,2000,";
    for (int8_t i = 0; i < CH_SIZE; i++)
      std::cout << "," << read(i);
    std::cout << std::endl;
  }
  void print() const {
    log_i("Reflector: %4d %4d %4d %4d", read(0), read(1), read(2), read(3));
  }

private:
  const std::array<int8_t, CH_SIZE> tx_pins; //< 赤外線LEDのピン
  const std::array<int8_t, CH_SIZE> rx_pins; //< フォトトランジスタのピン
  std::array<int16_t, CH_SIZE> value;        //< リフレクタの測定値
  TimerSemaphore ts; //< インターバル用タイマー
  ctrl::Accumulator<int, ave_num> buffer[CH_SIZE]; //< 平均計算用

  void update() {
    ts.take(); //< スタートを同期
    for (int i : {2, 1, 0, 3}) {
      ts.take(); //< 干渉防止のウエイト
      // uint16_t offset = analogRead(rx_pins[i]); //< オフセットを取得
      uint16_t offset = 0;                   //< オフセットを取得
      digitalWrite(tx_pins[i], HIGH);        //< 放電開始
      uint16_t raw = analogRead(rx_pins[i]); //< ADC取得
      digitalWrite(tx_pins[i], LOW);         //< 充電開始

      int temp = (int)raw - offset; //< オフセットとの差をとる
      if (temp < 1)
        temp = 1;           //< 0以下にならないように1で飽和
      buffer[i].push(temp); //< 保存
      value[i] = buffer[i].average();
    }
  }
  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();
    }
  }
};
