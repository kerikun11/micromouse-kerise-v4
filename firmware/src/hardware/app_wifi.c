/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "app_wifi.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>

#include <string.h>
#include <sys/time.h>

static const char *TAG = "WIFI";

/* FreeRTOS event group to signal when we are connected & ready to make a
 * request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

static esp_err_t event_handler(void *ctx, system_event_t *event) {
  ESP_LOGI(TAG, "WiFi event: %d", (int)event->event_id);
  switch (event->event_id) {
  case SYSTEM_EVENT_STA_START:
    esp_wifi_connect();
    break;
  case SYSTEM_EVENT_STA_GOT_IP:
    xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    esp_wifi_connect();
    xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    break;
  default:
    break;
  }
  return ESP_OK;
}

static wifi_config_t wifi_config;

void app_wifi_init(const char *ssid, const char *password) {
  tcpip_adapter_init();
  wifi_event_group = xEventGroupCreate();
  strcpy((char *)wifi_config.sta.ssid, ssid);
  strcpy((char *)wifi_config.sta.password, password);
  wifi_config.sta.listen_interval = 10;

  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  // ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
}

void app_wifi_wakeup() {
  esp_event_loop_set_cb(event_handler, NULL);
  ESP_ERROR_CHECK(esp_wifi_start());
  // ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
}

void app_wifi_suspend() {
  esp_event_loop_set_cb(NULL, NULL);
  ESP_ERROR_CHECK(esp_wifi_disconnect());
  ESP_ERROR_CHECK(esp_wifi_stop());
  xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
}

void app_wifi_deinit() { ESP_ERROR_CHECK(esp_wifi_deinit()); }

int app_wifi_wait_connected() {
  portTickType timeout = 10000 / portTICK_PERIOD_MS;
  EventBits_t uxBits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                                           false, true, timeout);
  if (uxBits & CONNECTED_BIT)
    return 0;
  return -1;
}
