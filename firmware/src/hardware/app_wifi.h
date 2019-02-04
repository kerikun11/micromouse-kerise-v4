/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef _APP_WIFI_H_
#define _APP_WIFI_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void app_wifi_init(const char *ssid, const char *password);
void app_wifi_deinit(void);

void app_wifi_wakeup(void);
void app_wifi_suspend(void);

int app_wifi_wait_connected(void);

int sntp_perform(void);

#ifdef __cplusplus
}
#endif

#endif
