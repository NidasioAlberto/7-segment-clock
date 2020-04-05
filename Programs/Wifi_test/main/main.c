#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define WIFI_SSID "Alberto"
#define WIFI_PSW "SEGoNtivERLIaldE"
#define MAXIMUM_RETRY 4

// Wifi events
static EventGroupHandle_t s_wifi_event_group;

// The event group allows multiple bits for each event, but we only care about one event
// are we connected to the AP with an IP?
const int WIFI_CONNECTED_BIT = BIT0;

static const char *TAG = "Wifi test";
static int s_retry_num = 0;

// Init the wifi in station mode
void wifi_init_sta();
static esp_err_t event_handler(void *ctx, system_event_t *event);

void app_main(void)
{
  //Initialize NVS for wifi use
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "Wifi connection initialising...");
  wifi_init_sta();
}

void wifi_init_sta()
{
  s_wifi_event_group = xEventGroupCreate();

  // Init the TCP/IP stack
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

  // Init the esp wifi
  wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

  // Set wifi station mode
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  // Set the wifi configuration
  wifi_config_t wifi_config = {
      .sta = {
          .ssid = WIFI_SSID,
          .password = WIFI_PSW},
  };
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

  // Start esp wifi
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Wifi initialization completed");
  ESP_LOGI(TAG, "Connecting to access point %s with password %s", WIFI_SSID, WIFI_PSW);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
  switch (event->event_id)
  {
  case SYSTEM_EVENT_STA_START:
    esp_wifi_connect();
    break;
  case SYSTEM_EVENT_STA_GOT_IP:
    ESP_LOGI(TAG, "Assigned ip: %s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    ESP_LOGI(TAG, "Connection to the AP failed\n");
    
    if (s_retry_num < MAXIMUM_RETRY)
    {
      esp_wifi_connect();
      xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
      s_retry_num++;
      ESP_LOGI(TAG, "Retry to connect to the AP");
    }
    break;
  default:
    break;
  }
  return ESP_OK;
}