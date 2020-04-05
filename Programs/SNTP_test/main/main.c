#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include <time.h>
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define WIFI_SSID "Alberto-phone"
#define WIFI_PSW "AleCheru"
#define MAXIMUM_RETRY 4

// To set the SNTP sync interval go in menuconfig > Component config > LWIP > SNTP

#define SNTP_SYNC_PERIOD 1 //in seconds

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

void time_sync_notification_cb(struct timeval *tv)
{
  ESP_LOGI(TAG, "Notification of a time synchronization event");

  time_t now;
  struct tm timeinfo;
  time(&now);

  char strftime_buf[64];

  // Set timezone to Eastern Standard Time and print local time
  setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
  tzset();
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  ESP_LOGI(TAG, "The current date/time in New York is: %s", strftime_buf);

  // Set timezone to China Standard Time
  setenv("TZ", "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", 1);
  tzset();
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  ESP_LOGI(TAG, "The current date/time in Rome is: %s", strftime_buf);
}

void app_main(void)
{
  //Initialize NVS for wifi use
  ESP_ERROR_CHECK(nvs_flash_init());

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

// Event handler for wifi events
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

    // Initialize SNTP
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "0.it.pool.ntp.org");
    sntp_setservername(1, "1.it.pool.ntp.org");
    sntp_setservername(2, "2.it.pool.ntp.org");
    sntp_setservername(3, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
    ESP_LOGI(TAG, "SNTP initialization completed");

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