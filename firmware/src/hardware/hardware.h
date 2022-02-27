#pragma once

/* Driver */
#include "hardware/buzzer.h"
#include "hardware/fan.h"
#include "hardware/led.h"
#include "hardware/motor.h"
/* Sensor */
#include "hardware/button.h"
#include "hardware/encoder.h"
#include "hardware/imu.h"
#include "hardware/reflector.h"
#include "hardware/tof.h"

namespace hardware {

class Hardware {
public:
  /* Driver */
  Buzzer *bz;
  LED *led;
  Motor *mt;
  Fan *fan;
  /* Sensor */
  Button *btn;
  IMU *imu;
  Encoder *enc;
  Reflector *rfl;
  ToF *tof;

private:
  static constexpr float thr_battery = 3.8f;

public:
  Hardware() {}
  bool init() {
    /* pullup all the pins of the SPI-CS so that the bus is not blocked */
    for (auto p : CONFIG_SPI_CS_PINS) {
      gpio_reset_pin(p);
      gpio_set_direction(p, GPIO_MODE_INPUT);
      gpio_pullup_en(p);
    }

    /* Buzzer (initialize first to notify errors by sound) */
    bz = Buzzer::get_instance();
    bz->init(BUZZER_PIN, BUZZER_LEDC_CHANNEL, BUZZER_LEDC_TIMER);
    /* Button */
    btn = new Button();
    btn->init(BUTTON_PIN);
    /* I2C for LED, ToF */
    if (!peripheral::I2C::install(I2C_PORT_NUM, I2C_SDA_PIN, I2C_SCL_PIN))
      bz->play(hardware::Buzzer::ERROR);
    /* LED */
    led = new LED();
    if (!led->init(I2C_PORT_NUM_LED))
      bz->play(hardware::Buzzer::ERROR);
    /* ADC for Battery, Reflector */
    if (!peripheral::ADC::init())
      bz->play(hardware::Buzzer::ERROR);

    /* Battery Check */
    if (batteryCheck())
      bz->play(hardware::Buzzer::BOOT);
    else
      bz->play(hardware::Buzzer::SHUTDOWN);

    /* SPI for IMU, Encoder */
    if (!peripheral::SPI::install(CONFIG_SPI_HOST, CONFIG_SPI_SCLK_PIN,
                                  CONFIG_SPI_MISO_PIN, CONFIG_SPI_MOSI_PIN,
                                  CONFIG_SPI_DMA_CHAIN))
      bz->play(hardware::Buzzer::ERROR);
    /* IMU */
    imu = new IMU();
    if (!imu->init(ICM20602_SPI_HOST, ICM20602_CS_PINS))
      bz->play(hardware::Buzzer::ERROR);
    /* Encoder */
    enc = new Encoder();
    if (!enc->init(ENCODER_SPI_HOST, ENCODER_CS_PINS))
      bz->play(hardware::Buzzer::ERROR);
    /* Reflector */
    rfl = new Reflector(REFLECTOR_TX_PINS, REFLECTOR_RX_CHANNELS);
    if (!rfl->init())
      bz->play(hardware::Buzzer::ERROR);
    /* ToF */
    tof = new ToF();
    if (!tof->init(I2C_PORT_NUM_TOF))
      bz->play(hardware::Buzzer::ERROR);
    /* Motor */
    mt = new Motor(MOTOR_L_CTRL1_PIN, MOTOR_L_CTRL2_PIN, MOTOR_R_CTRL1_PIN,
                   MOTOR_R_CTRL2_PIN);
    /* Fan */
    fan = new Fan(FAN_PIN);

    /* Ending */
    return true;
  }

  /**
   * @brief Sample the Battery Voltage
   * @return float voltage [V]
   */
  static float getBatteryVoltage() {
    // return 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
    return 2 * peripheral::ADC::read_milli_voltage(BAT_VOL_ADC1_CHANNEL, 10) /
           1e3f;
  }
  /**
   * @brief バッテリー電圧をLEDで表示
   * @param voltage [V]
   */
  void batteryLedIndicate(const float voltage) {
    led->set(0);
    if (voltage < 4.0f)
      led->set(0x01);
    else if (voltage < 4.1f)
      led->set(0x03);
    else if (voltage < 4.2f)
      led->set(0x07);
    else
      led->set(0x0F);
  }
  bool batteryCheck() {
    const float voltage = getBatteryVoltage();
    batteryLedIndicate(voltage);
    app_logi << "Battery Voltage: " << voltage << " [V]" << std::endl;
    if (voltage < thr_battery) {
      app_logw << "Battery Low!" << std::endl;
      return false;
    }
    return true;
  }
};

} // namespace hardware
