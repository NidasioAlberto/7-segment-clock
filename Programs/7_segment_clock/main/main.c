// Led display
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/pcnt.h"
#include "driver/timer.h"

// Wifi and SNTP
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include <time.h>

// Logs
#include "esp_log.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/**
 * This program tests the capabilities of the 7 segment 6 digit display
 * It shows a counter updating every second
*/

// TLC5940 pins
#define VPRG_PIN 32
#define SIN_PIN 33  // SPI_MOSI
#define SOUT_PIN 34 // SPI_MISO
#define SCLK_PIN 25 // SPI_CLK
#define XLAT_PIN 26
#define BLANK_PIN 27
#define DCPRG_PIN 14
#define GSCLK_PIN 12

// Gray scale clock feedback pin
#define COUNTER_PIN 35

// Line pins
#define LINE_1_PIN 19
#define LINE_2_PIN 21
#define LINE_3_PIN 18
#define LINES_NUMER 3

// Ledc control unit
#define APP_PCNT_UNIT PCNT_UNIT_0

// Gray scale task
#define GS_CYCLE_TASK_NAME "GS_CYCLE"
#define GS_CYCLE_TASK_PRIORITY 6 // 0 - 10

// Timer task
#define TIMER_TASK_NAME "TIMER"
#define TIMER_TASK_PRIORITY 2

// SPI, GSCLK and counter frequencies
#define GS_CLOCK_FREQ 5 * 1000 * 1000
#define SCLK_FREQ 5 * 1000 * 1000                           // SPI frequency
#define COUNTER_FREQ_MS 1000                                // Timer frequency in milliseconds
#define TIMER_DIVIDER 16                                    //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER / 1000) // convert counter value to milliseconds

// Segment name-number
#define SEGMENT_G_NUM 0
#define SEGMENT_F_NUM 5
#define SEGMENT_E_NUM 4
#define SEGMENT_D_NUM 3
#define SEGMENT_C_NUM 2
#define SEGMENT_B_NUM 1
#define SEGMENT_A_NUM 6

// Led status values 0-4095
#define LED_ON_VALUE 4095
#define LED_OFF_VALUE 0

// Wifi configuration
#define WIFI_SSID "Alberto"
#define WIFI_PSW "SEGoNtivERLIaldE"
#define MAXIMUM_RETRY 4

// SNTP configuration
#define SNTP_SERVER_0 "time-a-g.nist.gov"
#define SNTP_SERVER_1 "pool.ntp.org"
#define SNTP_SYNC_PERIOD 1 //in seconds

// Line counter
uint8_t line_counter = 0;

// Gray scale clock data (12 bits * 16 outputs = 192 bits)
uint8_t gs_data[24] = {0}; //startup value => led off (duty cycle 0)

// Interrupt service routine for pcnt
pcnt_isr_handle_t user_isr_handle = NULL;

// Spi device handle
spi_device_handle_t spi;

// Display values
uint16_t segment_0 = 0;
uint16_t segment_1 = 0;
uint16_t segment_2 = 0;

// Wifi events
static EventGroupHandle_t s_wifi_event_group;

// The event group allows multiple bits for each event, but we only care about one event
// are we connected to the AP with an IP?
const int WIFI_CONNECTED_BIT = BIT0;

static int s_retry_num = 0;

// Program TAG
static const char *TAG = "7-segment-clock";

// Functions
void tlc5940_init();
void display_value_timer_init();
void set_pin_value(uint8_t pin, uint16_t value);
void compute_leds_values();
void update_line_counter();
void wifi_init_sta();
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event);
void time_sync_notification_cb(struct timeval *tv);

// IRAM functions
static void IRAM_ATTR gs_clk_handler(void *args);
static void IRAM_ATTR timer_handler(void *args);

// Tasks
void gs_input_data_cycle_task(void *parameters);
void timer_task(void *parameters);

// Main
void app_main(void)
{
    // Initialize the wifi first
    wifi_init_sta();

    // Initialize the TLC5940
    tlc5940_init();

    // Initialize the timer
    display_value_timer_init();
}

// Wifi initialization
void wifi_init_sta()
{
    //Initialize NVS for wifi use
    ESP_ERROR_CHECK(nvs_flash_init());

    s_wifi_event_group = xEventGroupCreate();

    // Init the TCP/IP stack
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));

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

    //ESP_LOGI(TAG, "Wifi initialization completed");
    //ESP_LOGI(TAG, "Connecting to access point %s with password %s", WIFI_SSID, WIFI_PSW);
}

// Set the pins as showed in the programming flowchar of the TLC5940
void tlc5940_init()
{
    //gpio_set_direction(DCPRG_PIN, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(VPRG_PIN, GPIO_MODE_INPUT_OUTPUT); // Used to read the value of the pin (not from the device)
    gpio_set_direction(XLAT_PIN, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(BLANK_PIN, GPIO_MODE_DEF_OUTPUT);

    // Lines
    gpio_set_direction(LINE_1_PIN, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(LINE_2_PIN, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(LINE_3_PIN, GPIO_MODE_DEF_OUTPUT);

    // Set initial output value
    gpio_set_level(SCLK_PIN, 0);
    gpio_set_level(DCPRG_PIN, 0);
    gpio_set_level(VPRG_PIN, 0);
    gpio_set_level(BLANK_PIN, 0);
    gpio_set_level(LINE_1_PIN, 0);
    gpio_set_level(LINE_2_PIN, 0);
    gpio_set_level(LINE_3_PIN, 0);

    // Set the gray scale clock
    ledc_timer_config_t ledc_timer = {
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .freq_hz = GS_CLOCK_FREQ,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = GSCLK_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 1,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);

    // Setup the pulse counter
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = COUNTER_PIN,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 4095,
        .counter_l_lim = 0,
        .unit = APP_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0};
    pcnt_unit_config(&pcnt_config);
    pcnt_event_enable(APP_PCNT_UNIT, PCNT_EVT_H_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(APP_PCNT_UNIT);
    pcnt_counter_clear(APP_PCNT_UNIT);

    // Set interrupt
    pcnt_isr_register(gs_clk_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(APP_PCNT_UNIT);

    // Start the counter
    pcnt_counter_resume(APP_PCNT_UNIT);

    // Set spi master driver
    spi_bus_config_t bus_config = {
        .sclk_io_num = SCLK_PIN,
        .mosi_io_num = SIN_PIN,
        .miso_io_num = SOUT_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 24 * 8};

    //1: Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 0));

    // Spi defice configuration
    spi_device_interface_config_t device_config = {
        .clock_speed_hz = SCLK_FREQ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
        .input_delay_ns = 10,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &device_config, &spi));
}

// Setup the timer to update the value on the display
void display_value_timer_init()
{
    // Init the counter timer
    timer_config_t timer_config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = TIMER_DIVIDER,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);

    // Set to 0 the counter start value
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x0LLU);

    // Configure the alarm and the interrupt
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, COUNTER_FREQ_MS * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);

    // Start the timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// Handler for counter overflow at 4096 pulses
static void IRAM_ATTR gs_clk_handler(void *args)
{
    // Clear the interrupt, this is need to be done immediately otherwise the counter will be increased again to MAX+1 an the interrupt will be triggered another time
    PCNT.int_clr.val = BIT(APP_PCNT_UNIT);

    // Start the task that will handle the gs data input cycle
    xTaskCreatePinnedToCore(gs_input_data_cycle_task, GS_CYCLE_TASK_NAME, 4096, NULL, GS_CYCLE_TASK_PRIORITY, NULL, APP_CPU_NUM); // We'll work on the APP_CPU while the main default loop will work on the PRO_CPU
}

void gs_input_data_cycle_task(void *parameters)
{
    // Compute the leds values
    compute_leds_values();

    // Update and set the line counter
    update_line_counter();
    gpio_set_level(LINE_1_PIN, line_counter == 0);
    gpio_set_level(LINE_2_PIN, line_counter == 1);
    gpio_set_level(LINE_3_PIN, line_counter == 2);

    // PREVIOUS CYCLE

    //1: Set BLANK high to restart the GS cycle
    gpio_set_level(BLANK_PIN, 1);

    //2: Pulse XLAT to latch the previous values
    gpio_set_level(XLAT_PIN, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(XLAT_PIN, 0);

    // NEW CYCLE

    //3: Set BLANK low
    gpio_set_level(BLANK_PIN, 0);

    //4: Clock out the new gs value
    spi_transaction_t transaction = {
        .length = 24 * 8,
        .rxlength = 3 * 8,
        .tx_buffer = &gs_data,
        .user = (void *)0,
    };

    spi_device_polling_transmit(spi, &transaction);

    // Finally delete the task
    vTaskDelete(NULL);
}

static void IRAM_ATTR timer_handler(void *args)
{
    // Clear the interrupt
    TIMERG0.int_clr_timers.t0 = 1;

    // Re-enable the timer for the next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    xTaskCreatePinnedToCore(timer_task, TIMER_TASK_NAME, 4096, NULL, TIMER_TASK_PRIORITY, NULL, APP_CPU_NUM);
}

// Event handler for wifi events
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
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
        sntp_setservername(0, "it.pool.ntp.org");
        sntp_setservername(1, "pool.ntp.org");
        sntp_set_time_sync_notification_cb(time_sync_notification_cb);
        sntp_init();

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

void timer_task(void *parameters)
{
    // Retrieve the time
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    segment_0 = timeinfo.tm_sec;
    segment_1 = timeinfo.tm_min;
    segment_2 = timeinfo.tm_hour;

    ESP_LOGI(TAG, "Displayed time updated: %d:%d:%d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    // Finally delete the task
    vTaskDelete(NULL);
}

// Set pin value (pin from 0 to 15 and value from 0 to 4095)
void set_pin_value(uint8_t pin, uint16_t value)
{
    uint8_t mask1, mask2, index;
    uint8_t dataPt1 = 0;
    uint8_t dataPt2 = 0;

    // Return if input data are not valid
    if (pin > 15)
        return;
    if (value > 4095)
        value = 4095;

    pin = 15 - pin;
    index = pin + pin / 2;

    // Split the data into 8 bit element and define the mask
    dataPt1 |= value >> (pin % 2 == 0 ? 4 : 8);
    mask1 = (pin % 2 == 0 ? 0xff : 0x0f);
    mask2 = (pin % 2 == 0 ? 0xf0 : 0xff);
    dataPt2 |= value << (pin % 2 == 0 ? 4 : 0);

    // Apply the data
    gs_data[index] = (gs_data[index] & ~mask1) | dataPt1;
    gs_data[index + 1] = (gs_data[index + 1] & ~mask2) | dataPt2;
}

// Compute the TLC5940 pin values to display the proper value
void compute_leds_values()
{
    uint32_t display_value = 0;
    uint8_t digit1_value;
    uint8_t digit2_value;

    // Reset the led values
    for (uint8_t i = 0; i < 24; i++)
        gs_data[i] = 0;

    if (line_counter == 0)
        display_value = segment_0;

    if (line_counter == 1)
        display_value = segment_1;

    if (line_counter == 2)
        display_value = segment_2;

    digit1_value = display_value % 10;
    digit2_value = (display_value / 10) % 10;

    // DIGIT 1 of line

    // Find the correct value for all the pins

    // Pin 0 (segment A)
    if (digit1_value != 1 && digit1_value != 4)
        set_pin_value(SEGMENT_A_NUM + 7, LED_ON_VALUE);

    // Pin 1 (segment B)
    if (digit1_value != 5 && digit1_value != 6)
        set_pin_value(SEGMENT_B_NUM + 7, LED_ON_VALUE);

    // Pin 2 (segment C)
    if (digit1_value != 2)
        set_pin_value(SEGMENT_C_NUM + 7, LED_ON_VALUE);

    // Pin 3 (segment D)
    if (digit1_value != 1 && digit1_value != 4 && digit1_value != 7)
        set_pin_value(SEGMENT_D_NUM + 7, LED_ON_VALUE);

    // Pin 4 (segment E)
    if (digit1_value == 0 || digit1_value == 2 || digit1_value == 6 || digit1_value == 8)
        set_pin_value(SEGMENT_E_NUM + 7, LED_ON_VALUE);

    // Pin 5 (segment F)
    if (digit1_value != 1 && digit1_value != 2 && digit1_value != 3 && digit1_value != 7)
        set_pin_value(SEGMENT_F_NUM + 7, LED_ON_VALUE);

    // Pin 6 (segment G)
    if (digit1_value != 0 && digit1_value != 1 && digit1_value != 7)
        set_pin_value(SEGMENT_G_NUM + 7, LED_ON_VALUE);

    // DIGIT 2 of line

    // Find the correct value for all the pins

    // Pin 0 (segment A)
    if (digit2_value != 1 && digit2_value != 4)
        set_pin_value(SEGMENT_A_NUM, LED_ON_VALUE);

    // Pin 1 (segment B)
    if (digit2_value != 5 && digit2_value != 6)
        set_pin_value(SEGMENT_B_NUM, LED_ON_VALUE);

    // Pin 2 (segment C)
    if (digit2_value != 2)
        set_pin_value(SEGMENT_C_NUM, LED_ON_VALUE);

    // Pin 3 (segment D)
    if (digit2_value != 1 && digit2_value != 4 && digit2_value != 7)
        set_pin_value(SEGMENT_D_NUM, LED_ON_VALUE);

    // Pin 4 (segment E)
    if (digit2_value == 0 || digit2_value == 2 || digit2_value == 6 || digit2_value == 8)
        set_pin_value(SEGMENT_E_NUM, LED_ON_VALUE);

    // Pin 5 (segment F)
    if (digit2_value != 1 && digit2_value != 2 && digit2_value != 3 && digit2_value != 7)
        set_pin_value(SEGMENT_F_NUM, LED_ON_VALUE);

    // Pin 6 (segment G)
    if (digit2_value != 0 && digit2_value != 1 && digit2_value != 7)
        set_pin_value(SEGMENT_G_NUM, LED_ON_VALUE);

    set_pin_value(14, LED_ON_VALUE);
    set_pin_value(15, LED_ON_VALUE);
}

void update_line_counter()
{
    line_counter++;
    if (line_counter >= LINES_NUMER)
        line_counter = 0;
}

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

    // Test the time display
    segment_0 = timeinfo.tm_sec;
    segment_1 = timeinfo.tm_min;
    segment_2 = timeinfo.tm_hour;
    ESP_LOGI(TAG, "New time displayed: %d:%d:%d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}