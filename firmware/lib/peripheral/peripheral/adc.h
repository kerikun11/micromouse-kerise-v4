/**
 * @file adc.h
 * @brief ADC for ESP32
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-28
 */
#pragma once

#include <driver/adc.h>
#include <esp_adc_cal.h>

namespace peripheral {

class ADC {
public:
  static bool init() {
    ESP_ERROR_CHECK(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF));
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_12Bit));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &chars);
    return true;
  }
  static int read_raw(adc1_channel_t channel) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc1_config_channel_atten(channel, atten));
    return adc1_get_raw(channel);
  }
  static int read_milli_voltage(adc1_channel_t channel,
                                int num_average_samples = 1) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc1_config_channel_atten(channel, atten));
    uint32_t adc_reading = 0;
    for (int i = 0; i < num_average_samples; i++)
      adc_reading += adc1_get_raw(channel);
    adc_reading /= num_average_samples;
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &chars);
    return voltage;
  }

private:
  static const int DEFAULT_VREF = 1100;
  static const adc_unit_t unit = ADC_UNIT_1;
  static const adc_atten_t atten = ADC_ATTEN_DB_11;
  static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
  static esp_adc_cal_characteristics_t chars;
};

}; // namespace peripheral
