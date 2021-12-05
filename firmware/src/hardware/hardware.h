#pragma once

#include "machine/global.h"
#include "peripheral/spiffs.h"

namespace hardware {

class Hardware {
public:
  static bool init() {
    /* pullup all the pins of the SPI-CS so that the bus is not blocked  */
    for (auto p : CONFIG_SPI_CS_PINS) {
      gpio_set_direction(p, GPIO_MODE_INPUT);
      gpio_pullup_en(p);
    }

    /* Buzzer (initialize first to notify errors by sound) */
    bz.init(BUZZER_PIN, BUZZER_LEDC_CHANNEL, BUZZER_LEDC_TIMER);
    /* I2C for LED, ToF */
    if (!peripheral::I2C::install(I2C_PORT_NUM, I2C_SDA_PIN, I2C_SCL_PIN))
      bz.play(Buzzer::ERROR);
    /* LED */
    if (!led.init())
      bz.play(Buzzer::ERROR);
    /* ADC for Battery, Reflector */
    if (!peripheral::ADC::init())
      bz.play(Buzzer::ERROR);

    /* Battery Check */
    ui.batteryCheck();

    /* Normal Boot */
    bz.play(Buzzer::BOOT);

    /* SPIFFS for MazeRobot, WallDetector */
    if (!peripheral::SPIFFS::init())
      bz.play(Buzzer::ERROR);
    /* SPI for IMU, Encoder */
    if (!peripheral::SPI::install(CONFIG_SPI_HOST, CONFIG_SPI_SCLK_PIN,
                                  CONFIG_SPI_MISO_PIN, CONFIG_SPI_MOSI_PIN,
                                  CONFIG_SPI_DMA_CHAIN))
      bz.play(Buzzer::ERROR);
    /* IMU */
    if (!imu.init(ICM20602_SPI_HOST, ICM20602_CS_PINS))
      bz.play(Buzzer::ERROR);
    /* Encoder */
    if (!enc.init(ENCODER_SPI_HOST, ENCODER_CS_PINS))
      bz.play(Buzzer::ERROR);
    /* Reflector */
    if (!ref.init())
      bz.play(Buzzer::ERROR);
    /* ToF */
    if (!tof.init())
      bz.play(Buzzer::ERROR);

    return true;
  }
};

} // namespace hardware
