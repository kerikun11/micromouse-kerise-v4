/*
 * ALL CODE BELOW WAS "STOLEN" FROM ORIGINAL REPOSITORY
 * https://github.com/espressif/arduino-esp32 FUNCTIONS adcStart/adcBusy/adcEnd
 * WAS REMOVED AFTER 1.0.4 RELEASE OF "Arduino core for the esp32" BUT THESE
 * FUNCTIONS ARE VERY USEFUL FOR "FINE TUNNING" AND "NO-WAIT" ADC READING. I
 * JUST COPY CODE FROM .c/.h AND TO USE IT IN MY LIBRARY WITHOUT ANY
 * MODIFICATION star0413@gmail.com Ilia Starkov
 */
/*
 * Arduino.h - Main include file for the Arduino SDK Copyright (c) 2005-2013
 * Arduino Team.  All right reserved.  This library is free software; you can
 * redistribute it and/or modify it under the terms of the GNU Lesser General
 * Public License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.  This
 * library is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.  You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp32-hal.h"

/*
 * Start ADC conversion on attached pin's bus
 */
bool adcStart(uint8_t pin);

/*
 * Check if conversion on the pin's ADC bus is currently running
 */
bool adcBusy(uint8_t pin);

/*
 * Get the result of the conversion (will wait if it have not finished)
 */
uint16_t adcEnd(uint8_t pin);

#ifdef __cplusplus
}
#endif
