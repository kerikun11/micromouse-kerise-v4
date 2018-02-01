/**
  KERISE v3-2
  Author:  kerikun11 (Github: kerikun11)
  Date:    2017.10.25
*/

#include <WiFi.h>
#include "config.h"

#include "imu.h"
IMU imu;

void setup() {
  WiFi.mode(WIFI_OFF);
  Serial.begin(2000000);
  printf("\n**************** KERISE ****************\n");
  pinMode(AS5048A_CS_PIN, INPUT_PULLUP);
  pinMode(ICM20602_L_CS_PIN, INPUT_PULLUP);
  pinMode(ICM20602_L_CS_PIN, INPUT_PULLUP);

  imu.begin(ICM20602_SPI_HOST, ICM20602_CS_PINS, true, ICM20602_SCLK_PIN, ICM20602_MISO_PIN, ICM20602_MOSI_PIN, ICM20602_SPI_DMA_CHAIN);
  imu.calibration();
}

void loop() {
  imu.print();
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

