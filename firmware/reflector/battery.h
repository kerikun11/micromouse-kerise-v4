#pragma once

class Battery {
  public:
    Battery(int8_t pin): pin(pin) {}
    bool check() {
      float voltage = getVoltage();
      log_i("Battery Voltage: %.3f", voltage);
      if (voltage < 3.8f) {
        log_w("Battery Low!");
        return false;
      }
      return true;
    }
    void deepsleep() {
      delay(3000);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
      esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
      esp_deep_sleep_start();
    }
    float getVoltage() {
      pinMode(pin, ANALOG);
      return 2 * 1.1f * 3.54813389f * analogRead(BAT_VOL_PIN) / 4095;
    }
  private:
    int8_t pin;
};

