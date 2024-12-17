/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <cstdint>
#include <cstring>
#include <stdio.h>

#include "mqtt_manager.h"
#include "wifi_manager.h"

static const char* TAG = "main";

char message[512];

extern "C" void app_main(void)
{
    vTaskDelay(300);

    esp_pm_config_t pm_config = {80, 10, true};

    esp_pm_configure(&pm_config);

    wifi_start();

    ESP_LOGI(TAG, "Wifi Started");

    while (!wifi_is_connected()) {
        wifi_connect();
        vTaskDelay(1000);
        ESP_LOGI(TAG, "Connecting to WiFi");
    }

    mqtt_start();

    memset(message, 0, sizeof(message));

    uint32_t count = 0;

    uint32_t curr_time = esp_timer_get_time();

    while (true) {
        count++;

        while (!wifi_is_connected()) {
            wifi_connect();
            vTaskDelay(1000);
            ESP_LOGI(TAG, "Connecting to WiFi");
        }

        mqtt_send_message((char*)message, sizeof(message));

        vTaskDelay(2);
    }
}