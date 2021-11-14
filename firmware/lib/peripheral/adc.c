/*
 * ALL CODE BELOW WAS "STOLEN" FROM ORIGINAL REPOSITORY
 * https://github.com/espressif/arduino-esp32 FUNCTIONS adcStart/adcBusy/adcEnd
 * WAS REMOVED AFTER 1.0.4 RELEASE OF "Arduino core for the esp32" BUT THESE
 * FUNCTIONS ARE VERY USEFUL FOR "FINE TUNNING" AND "NO-WAIT" ADC READING. I
 * JUST COPY CODE FROM .c/.h AND TO USE IT IN MY LIBRARY WITHOUT ANY
 * MODIFICATION star0413@gmail.com Ilia Starkov
 */
// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "peripheral/adc.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"

static uint8_t __aAttenuation = 3; // 11db
static uint8_t __aWidth = 3;       // 12 bits
static uint8_t __aCycles = 8;
static uint8_t __aSamples = 0; // 1 sample
static uint8_t __aClockDiv = 1;

// Width of returned answer ()
static uint8_t __aReturnedWidth = 12;

void __aSetWidth(uint8_t bits) {
  if (bits < 9) {
    bits = 9;
  } else if (bits > 12) {
    bits = 12;
  }
  __aReturnedWidth = bits;
  __aWidth = bits - 9;
  SET_PERI_REG_BITS(SENS_SAR_START_FORCE_REG, SENS_SAR1_BIT_WIDTH, __aWidth, SENS_SAR1_BIT_WIDTH_S);
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_BIT, __aWidth, SENS_SAR1_SAMPLE_BIT_S);

  SET_PERI_REG_BITS(SENS_SAR_START_FORCE_REG, SENS_SAR2_BIT_WIDTH, __aWidth, SENS_SAR2_BIT_WIDTH_S);
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_SAMPLE_BIT, __aWidth, SENS_SAR2_SAMPLE_BIT_S);
}

void __aSetCycles(uint8_t cycles) {
  __aCycles = cycles;
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_CYCLE, __aCycles, SENS_SAR1_SAMPLE_CYCLE_S);
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_SAMPLE_CYCLE, __aCycles, SENS_SAR2_SAMPLE_CYCLE_S);
}

void __aSetSamples(uint8_t samples) {
  if (!samples) {
    return;
  }
  __aSamples = samples - 1;
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_NUM, __aSamples, SENS_SAR1_SAMPLE_NUM_S);
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_SAMPLE_NUM, __aSamples, SENS_SAR2_SAMPLE_NUM_S);
}

void __aSetClockDiv(uint8_t clockDiv) {
  if (!clockDiv) {
    return;
  }
  __aClockDiv = clockDiv;
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_CLK_DIV, __aClockDiv, SENS_SAR1_CLK_DIV_S);
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_CLK_DIV, __aClockDiv, SENS_SAR2_CLK_DIV_S);
}

void __aSetAttenuation(adc_attenuation_t attenuation) {
  __aAttenuation = attenuation & 3;
  uint32_t att_data = 0;
  int i = 10;
  while (i--) {
    att_data |= __aAttenuation << (i * 2);
  }
  WRITE_PERI_REG(SENS_SAR_ATTEN1_REG, att_data & 0xFFFF); // ADC1 has 8 channels
  WRITE_PERI_REG(SENS_SAR_ATTEN2_REG, att_data);
}

void IRAM_ATTR __aInit() {
  static bool initialized = false;
  if (initialized) {
    return;
  }

  __aSetAttenuation(__aAttenuation);
  __aSetCycles(__aCycles);
  __aSetSamples(__aSamples + 1); // in samples
  __aSetClockDiv(__aClockDiv);
  __aSetWidth(__aWidth + 9); // in bits

  SET_PERI_REG_MASK(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DATA_INV);
  SET_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DATA_INV);

  SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_FORCE_M); // SAR ADC1 controller (in RTC) is started by SW
  SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD_FORCE_M); // SAR ADC1 pad enable bitmap is controlled by SW
  SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_FORCE_M); // SAR ADC2 controller (in RTC) is started by SW
  SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD_FORCE_M); // SAR ADC2 pad enable bitmap is controlled by SW

  CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_M); // force XPD_SAR=0, use XPD_FSM
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_AMP, 0x2, SENS_FORCE_XPD_AMP_S); // force XPD_AMP=0

  CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_CTRL_REG, 0xfff << SENS_AMP_RST_FB_FSM_S); // clear FSM
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT1, 0x1, SENS_SAR_AMP_WAIT1_S);
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT2, 0x1, SENS_SAR_AMP_WAIT2_S);
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_SAR_AMP_WAIT3, 0x1, SENS_SAR_AMP_WAIT3_S);
  while (GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR1_REG, 0x7, SENS_MEAS_STATUS_S) != 0) ; // wait det_fsm==

  initialized = true;
}

bool IRAM_ATTR __adcStart(uint8_t pin) {

  int8_t channel = digitalPinToAnalogChannel(pin);
  if (channel < 0) {
    return false; // not adc pin
  }

  if (channel > 9) {
    channel -= 10;
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD, (1 << channel), SENS_SAR2_EN_PAD_S);
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
  } else {
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
  }
  return true;
}

bool IRAM_ATTR __adcBusy(uint8_t pin) {
  int8_t channel = digitalPinToAnalogChannel(pin);
  if (channel < 0) {
    return false; // not adc pin
  }

  if (channel > 7) {
    return (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0);
  }
  return (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0);
}

uint16_t IRAM_ATTR __adcEnd(uint8_t pin) {

  uint16_t value = 0;
  int8_t channel = digitalPinToAnalogChannel(pin);

  if (channel < 0) {
    return 0; // not adc pin
  }

  //  log_i("pin: %d", pin);

  if (channel > 7) {
    while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0) ; // wait for conversion
    value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
  } else {
    while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0) ; // wait for conversion
    value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
  }

  // Shift result if necessary
  uint8_t from = __aWidth + 9;

  if (from == __aReturnedWidth) {
    return value;
  }

  if (from > __aReturnedWidth) {
    return value >> (from - __aReturnedWidth);
  }

  return value << (__aReturnedWidth - from);
}

extern bool adcStart(uint8_t pin) __attribute__((weak, alias("__adcStart")));
extern bool adcBusy(uint8_t pin) __attribute__((weak, alias("__adcBusy")));
extern uint16_t adcEnd(uint8_t pin) __attribute__((weak, alias("__adcEnd")));
