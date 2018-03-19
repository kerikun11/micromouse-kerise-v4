/**
  KERISE v3-2
  Author:  kerikun11 (Github: kerikun11)
  Date:    2017.10.25
*/

#include <WiFi.h>
#include "config.h"

#include "imu.h"
IMU imu;

#include "encoder.h"
Encoder enc;

void setup() {
  WiFi.mode(WIFI_OFF);
  Serial.begin(2000000);
  printf("\n**************** KERISE ****************\n");
  pinMode(AS5048A_CS_PIN, INPUT_PULLUP);
  pinMode(ICM20602_L_CS_PIN, INPUT_PULLUP);
  pinMode(ICM20602_R_CS_PIN, INPUT_PULLUP);

  imu.begin(ICM20602_SPI_HOST, ICM20602_CS_PINS, true, ICM20602_SCLK_PIN, ICM20602_MISO_PIN, ICM20602_MOSI_PIN, ICM20602_SPI_DMA_CHAIN);
  enc.begin(AS5048A_SPI_HOST, AS5048A_CS_PIN, false, AS5048A_SCLK_PIN, AS5048A_MISO_PIN, AS5048A_MOSI_PIN, AS5048A_SPI_DMA_CHAIN);

  imu.calibration();
}

void loop() {
  imu.print();
  enc.print();
  delay(100);
  if (Serial.available()) {
    switch (Serial.read()) {
      case 't':
        imu.calibration();
        break;
      default:
        break;
    }
  }
}

