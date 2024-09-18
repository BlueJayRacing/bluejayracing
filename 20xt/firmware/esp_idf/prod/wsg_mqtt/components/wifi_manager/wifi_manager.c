#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"

#include "wifi_manager.h"

#define WIFI_CONNECTED_BIT BIT0

#define WIFI_SSID                       "bjr_wireless_axle_host" 
#define WIFI_PASSWORD                   "bluejayracing"

static const char* TAG = "wifi_manager";

static EventGroupHandle_t s_wifi_event_group;

static esp_netif_t *tutorial_netif = NULL;

static bool is_wifi_connected = false;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        is_wifi_connected = false;
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(TAG, "got IP\n");
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        is_wifi_connected = true;
    }
}

esp_err_t wifi_start(void)
{
#ifdef CONFIG_IDF_TARGET_ESP32C6
    gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_3, 0);

    gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_14, 1);
#endif

    esp_wifi_deinit();

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(nvs_flash_init());
    
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    tutorial_netif = esp_netif_create_default_wifi_sta();
    if (tutorial_netif == NULL)
    {
        ESP_LOGI(TAG, "Failed to create default WiFi STA interface");
        return ESP_FAIL;
    }

    esp_event_handler_register(WIFI_EVENT,
                                ESP_EVENT_ANY_ID,
                                wifi_event_handler,
                                NULL);

    esp_event_handler_register(IP_EVENT,
                                IP_EVENT_STA_GOT_IP,
                                wifi_event_handler,
                                NULL);

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();

    esp_err_t err = esp_wifi_init(&wifi_init_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize WiFi (err: %d)\n", err);
        return err;
    }
    ESP_LOGI(TAG, "Initialized Wifi\n");

    err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set WiFi Mode (err: %d)\n", err);
        return err;
    }
    ESP_LOGI(TAG, "Set WiFi Mode\n");

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set WiFi Mode (err: %d)\n", err);
        return err;
    }
    ESP_LOGI(TAG, "Set WiFi Mode\n");

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config_t));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
    wifi_config.sta.scan_method = WIFI_FAST_SCAN;

    memcpy(&wifi_config.sta.ssid, WIFI_SSID, sizeof(WIFI_SSID));
    memcpy(&wifi_config.sta.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set WiFi configuration (err: %d)\n", err);
        return err;
    }
    ESP_LOGI(TAG, "Set WiFi configuration\n");

    err = esp_wifi_start();

    ESP_LOGI(TAG, "Started wifi\n");

    return err;
}

esp_err_t wifi_connect(void)
{

    if (!is_wifi_connected)
    {
        return esp_wifi_connect();
    }

    return ESP_OK;
}

bool wifi_is_connected(void)
{
    return is_wifi_connected;
}