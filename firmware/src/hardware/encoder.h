#pragma once

#include <cmath>
#include <driver/spi_master.h>
#include <esp_err.h>

#include "config/model.h"

#if KERISE_SELECT == 4 || KERISE_SELECT == 3

#define ENCODER_STACK_SIZE 4096
#define ENCODER_PRIORITY 5

class Encoder {
public:
  static constexpr int ENCODER_PULSES = 16384;
#ifndef M_PI
  static constexpr float M_PI = 3.1415926535897932384626433832795f;
#endif

public:
  Encoder(float gear_ratio, float wheel_diameter)
      : gear_ratio(gear_ratio), wheel_diameter(wheel_diameter) {
    sampling_end_semaphore = xSemaphoreCreateBinary();
  }
  bool begin(spi_host_device_t spi_host, int8_t pin_cs) {
    // ESP-IDF SPI device initialization
    static spi_device_interface_config_t dev_cfg;
    dev_cfg.command_bits = 1;
    dev_cfg.address_bits = 0;
    dev_cfg.dummy_bits = 0;
    dev_cfg.mode = 1;
    dev_cfg.duty_cycle_pos = 0;
    dev_cfg.cs_ena_pretrans = 0;
    dev_cfg.cs_ena_posttrans = 0;
    dev_cfg.clock_speed_hz = 10000000;
    dev_cfg.spics_io_num = pin_cs;
    dev_cfg.flags = 0;
    dev_cfg.queue_size = 1;
    dev_cfg.pre_cb = NULL;
    dev_cfg.post_cb = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &encoder_spi));
    xTaskCreate([](void *obj) { static_cast<Encoder *>(obj)->task(); },
                "Encoder", ENCODER_STACK_SIZE, this, ENCODER_PRIORITY, NULL);
    return true;
  }
  float position(uint8_t ch) {
    float value = ((float)pulses_ovf[ch] * ENCODER_PULSES + pulses[ch]) *
                  wheel_diameter * M_PI * gear_ratio / ENCODER_PULSES;
    if (ch == 1)
      value = -value;
    return value;
  }
  int getPulses(uint8_t ch) {
    int value = pulses_ovf[ch] * ENCODER_PULSES + pulses[ch];
    if (ch == 1)
      value = -value;
    return value;
  }
  int getRaw(uint8_t ch) {
    int value = pulses[ch];
    if (ch == 1)
      value = -value;
    return value;
  }
  void clearOffset() {
    pulses_ovf[0] = 0;
    pulses_ovf[1] = 0;
  }
  void print() { log_d("Encoder L:\t%f\tR:\t%f\n", position(0), position(1)); }
  void csv() {
    //      printf("0,%d,%d,%d,%d\n", ENCODER_PULSES, -ENCODER_PULSES,
    //      getRaw(0), getRaw(1)); printf("0,%d,%d,%d,%d\n", ENCODER_PULSES,
    //      -ENCODER_PULSES, getPulses(0), getPulses(1));
    printf("0,%f,%f\n", position(0), position(1));
  }
  void samplingSemaphoreTake(TickType_t xBlockTime = portMAX_DELAY) {
    xSemaphoreTake(sampling_end_semaphore, xBlockTime);
  }

private:
  spi_device_handle_t encoder_spi;
  SemaphoreHandle_t sampling_end_semaphore;
  const float gear_ratio;
  const float wheel_diameter;
  int pulses[2];
  int pulses_prev[2];
  int pulses_ovf[2];

  void update() {
    uint8_t rxbuf[4];
    static spi_transaction_t tx;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.tx_data[0] = 0xFF;
    tx.tx_data[1] = 0xFF;
    tx.tx_data[2] = 0xFF;
    tx.tx_data[3] = 0xFF;
    tx.rx_buffer = rxbuf;
    tx.length = 32;
    ESP_ERROR_CHECK(spi_device_transmit(encoder_spi, &tx));

    pulses[0] = ((uint16_t)(0x3F & (rxbuf[0])) << 8) | rxbuf[1];
    pulses[1] = ((uint16_t)(0x3F & (rxbuf[2])) << 8) | rxbuf[3];
    for (int i = 0; i < 2; i++) {
      if (pulses[i] > pulses_prev[i] + ENCODER_PULSES / 2) {
        pulses_ovf[i]--;
      } else if (pulses[i] < pulses_prev[i] - ENCODER_PULSES / 2) {
        pulses_ovf[i]++;
      }
      pulses_prev[i] = pulses[i];
    }
  }

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();
      xSemaphoreGive(sampling_end_semaphore);
    }
  }
};

#elif KERISE_SELECT == 5

class MA730 {
public:
  static constexpr int MA730_PULSES = 16384;
#ifndef M_PI
  static constexpr float M_PI = 3.1415926535897932384626433832795f;
#endif

public:
  MA730(float gear_ratio, float wheel_diameter)
      : gear_ratio(gear_ratio), wheel_diameter(wheel_diameter) {}
  bool begin(spi_host_device_t spi_host, gpio_num_t pin_cs) {
    this->pin_cs = pin_cs;
    gpio_set_direction(pin_cs, GPIO_MODE_OUTPUT);
    gpio_set_level(pin_cs, 1);
    // ESP-IDF SPI device initialization
    static spi_device_interface_config_t dev_cfg;
    dev_cfg.command_bits = 0;
    dev_cfg.address_bits = 0;
    dev_cfg.dummy_bits = 0;
    dev_cfg.mode = 3;
    dev_cfg.duty_cycle_pos = 0;
    dev_cfg.cs_ena_pretrans = 0;
    dev_cfg.cs_ena_posttrans = 0;
    dev_cfg.clock_speed_hz = 20000000;
    dev_cfg.input_delay_ns = 0;
    dev_cfg.spics_io_num = pin_cs;
    dev_cfg.flags = 0;
    dev_cfg.queue_size = 1;
    dev_cfg.pre_cb = NULL;
    dev_cfg.post_cb = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &encoder_spi));
    return true;
  }
  float position() {
    return ((float)pulses_ovf * MA730_PULSES + pulses) * wheel_diameter * M_PI *
           gear_ratio / MA730_PULSES;
  }
  int getPulses() { return pulses_ovf * MA730_PULSES + pulses; }
  int getRaw() { return pulses; }
  void clearOffset() { pulses_ovf = 0; }
  void update() {
    uint8_t rxbuf[4];
    static spi_transaction_t tx;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.user = this;
    tx.tx_data[0] = 0x00;
    tx.tx_data[1] = 0x00;
    tx.rx_buffer = rxbuf;
    tx.length = 16;
    ESP_ERROR_CHECK(spi_device_transmit(encoder_spi, &tx)); //< send command
    ESP_ERROR_CHECK(spi_device_transmit(encoder_spi, &tx)); //< read data

    pulses = ((uint16_t)rxbuf[0] << 6) | (rxbuf[1] >> 2);
    if (pulses > pulses_prev + MA730_PULSES / 2) {
      pulses_ovf--;
    } else if (pulses < pulses_prev - MA730_PULSES / 2) {
      pulses_ovf++;
    }
    pulses_prev = pulses;
  }

private:
  spi_device_handle_t encoder_spi = NULL;
  gpio_num_t pin_cs;
  int pulses;
  int pulses_prev;
  int pulses_ovf;
  const float gear_ratio;
  const float wheel_diameter;
};

#define ENCODER_STACK_SIZE 4096
#define ENCODER_PRIORITY 5

class Encoder {
public:
  Encoder(float gear_ratio, float wheel_diameter)
      : ma({MA730(gear_ratio, wheel_diameter),
            MA730(gear_ratio, wheel_diameter)}) {
    sampling_end_semaphore = xSemaphoreCreateBinary();
  }
  bool begin(spi_host_device_t spi_host, std::array<gpio_num_t, 2> pins_cs) {
    if (!ma[0].begin(spi_host, pins_cs[0])) {
      log_e("Encoder L begin failed :(");
      return false;
    }
    if (!ma[1].begin(spi_host, pins_cs[1])) {
      log_e("Encoder R begin failed :(");
      return false;
    }
    xTaskCreate([](void *obj) { static_cast<Encoder *>(obj)->task(); },
                "Encoder", ENCODER_STACK_SIZE, this, ENCODER_PRIORITY, NULL);
    return true;
  }
  float position(uint8_t ch) {
    auto value = ma[ch].position();
    if (ch == 0)
      return -value;
    return value;
  }
  int getPulses(uint8_t ch) {
    auto value = ma[ch].getPulses();
    if (ch == 0)
      return -value;
    return value;
  }
  int getRaw(uint8_t ch) {
    auto value = ma[ch].getRaw();
    if (ch == 0)
      return -value;
    return value;
  }
  void clearOffset() {
    ma[0].clearOffset();
    ma[1].clearOffset();
  }
  void print() { log_d("Encoder L:\t%f\tR:\t%f\n", position(0), position(1)); }
  void csv() {
    printf("0,%d,%d,%d,%d\n", MA730::MA730_PULSES, -MA730::MA730_PULSES,
           getRaw(0), getRaw(1));
  }
  void samplingSemaphoreTake(TickType_t xBlockTime = portMAX_DELAY) {
    xSemaphoreTake(sampling_end_semaphore, xBlockTime);
  }

private:
  MA730 ma[2];
  SemaphoreHandle_t sampling_end_semaphore;

  void update() {
    ma[0].update();
    ma[1].update();
  }

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();
      xSemaphoreGive(sampling_end_semaphore);
    }
  }
};

#endif
