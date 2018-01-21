#pragma once

#include <Arduino.h>
#include <Wire.h>

#define PCA9632_DEV_ID 0x62 //全体制御用のI2Cアドレス

class LED {
  public:
    LED(const int pin_sda, const int pin_scl): pin_sda(pin_sda), pin_scl(pin_scl) {}
    bool begin(bool initializeWire = false) {
      if (initializeWire) Wire.begin(pin_sda, pin_scl);
      writeReg(0x00, B10000001);
      return true;
    }
    operator uint8_t() const {
      return value;
    }
    uint8_t operator=(uint8_t new_value) {
      value = new_value;
      uint8_t data = 0;
      data |= (value & 1) << 0;
      data |= (value & 2) << 1;
      data |= (value & 4) << 2;
      data |= (value & 8) << 3;
      writeReg(0x08, data);
      return value;
    }
  private:
    const int pin_sda, pin_scl;
    uint8_t value;
    void writeReg(uint8_t reg, uint8_t data) {
      Wire.beginTransmission(PCA9632_DEV_ID);//送信先のアドレスを指定して書き込み用に通信開始
      Wire.write(reg);//データを書き込むレジスタアドレスを送信
      Wire.write(data);//データを送信
      Wire.endTransmission(); //通信終了
    }
};

