/**
 * @file battery_monitor.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief バッテリーの電圧を測定し，残量を算出するツール
 * @version 0.1
 * @date 2018-12-18
 *
 * @copyright Copyright (c) 2018 Ryotaro Onuki
 *
 */
#pragma once

#include <driver/adc.h>

class BatteryMonitor {
public:
  BatteryMonitor(adc1_channel_t batteryAdcChannel, float batteryVoltageEmpty,
                 float batteryVoltageFull)
      : batteryAdcChannel(batteryAdcChannel), v_min(batteryVoltageEmpty),
        v_max(batteryVoltageFull) {}
  float getVoltage() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(batteryAdcChannel, ADC_ATTEN_DB_11);
    int adc_raw = adc1_get_raw(batteryAdcChannel);
    float voltage = 2 * 1.1f * 3.54813389f * adc_raw / 4095;
    return voltage;
  }
  uint8_t calcBatteryLevel(const float voltage) const {
    float v = voltage;
    if (v > v_max)
      return 100;
    else if (v > v_min)
      return ((v - v_min) / (v_max - v_min)) * 100;
    else
      return 0;
  }

private:
  adc1_channel_t batteryAdcChannel;
  const float v_min;
  const float v_max;
};
