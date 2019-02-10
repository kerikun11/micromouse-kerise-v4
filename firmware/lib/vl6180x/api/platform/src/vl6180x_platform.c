#include "vl6180x_platform.h"

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

/**
 * @brief execute delay in all polling api calls : @a VL6180x_RangePollMeasurement() and @a VL6180x_AlsPollMeasurement()
 *
 * A typical multi-thread or RTOs implementation is to sleep the task for some 5ms (with 100Hz max rate faster polling is not needed).
 * if nothing specific is needed, you can define it as an empty/void macro
 * @code
 * #define VL6180x_PollDelay(...) (void)0
 * @endcode
 * @param dev The device
 * @ingroup api_platform
 */
void VL6180x_PollDelay(VL6180xDev_t dev)
{
    vTaskDelay(1 / portTICK_PERIOD_MS);
}
