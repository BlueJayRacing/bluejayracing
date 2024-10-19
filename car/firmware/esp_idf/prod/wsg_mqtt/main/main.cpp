/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <cstdint>
#include <stdio.h>

#include "ad5626.hpp"
#include "ads1120.hpp"

#include "mqtt_manager.h"
#include "wifi_manager.h"

#ifdef CONFIG_IDF_TARGET_ESP32C3
#define SPI_MOSI_PIN 10
#define SPI_MISO_PIN 9
#define SPI_SCLK_PIN 8

#define ADC_CS_PIN   5
#define ADC_DRDY_PIN 4
#endif

#ifdef CONFIG_IDF_TARGET_ESP32C6
#define SPI_MOSI_PIN 18
#define SPI_MISO_PIN 20
#define SPI_SCLK_PIN 19

#define ADC_CS_PIN   21
#define ADC_DRDY_PIN 2
#endif

#define RTOS_QUEUE_SIZE 10

esp_err_t start_adc(ADS1120& adc);

static mqtt_time_t curr_time = {0, 0};

static const char* TAG = "main";

static TaskHandle_t read_adc_handle;

static TaskHandle_t send_mqtt_handle;

static QueueHandle_t data_queue = NULL;

void print_bytes(const char* data, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        printf("%02X ", (unsigned char)data[i]);
    }
    printf("\n");
}

static void send_mqtt_task(void* arg)
{
    ESP_LOGI(TAG, "Starting Send MQTT Task\n");

    esp_task_wdt_add(NULL);

    while (!wifi_is_connected()) {
        wifi_connect();
        vTaskDelay(10000);
        esp_task_wdt_reset();
    }

    mqtt_start();

    mqtt_message_t message;

    uint32_t count = 0;

    while (true) {
        if (uxQueueMessagesWaiting(data_queue) > 0) {
            xQueueReceive(data_queue, &message, 10);

            count++;

            while (!wifi_is_connected()) {
                wifi_connect();
                vTaskDelay(10000);
                esp_task_wdt_reset();
            }

            if (mqtt_can_send_received()) {
                mqtt_send_message((char*)&message, sizeof(mqtt_message_t));
            }

            mqtt_update_time(&curr_time);

            if (count % 40 == 0) {
                ESP_LOGI(TAG, "Got 40 messages: %d, micro_time : %llu\n", (int)xTaskGetTickCount(),
                         curr_time.pi_time_micro);
                esp_task_wdt_reset();
            }
        }
        vTaskDelay(5);
    }
}

static void read_adc_task(void* arg)
{
    esp_task_wdt_add(NULL);

    ESP_LOGI(TAG, "Starting Read ADC Task\n");

    spi_bus_config_t spi_cfg;
    memset(&spi_cfg, 0, sizeof(spi_bus_config_t));

    spi_cfg.mosi_io_num   = SPI_MOSI_PIN;
    spi_cfg.miso_io_num   = SPI_MISO_PIN;
    spi_cfg.sclk_io_num   = SPI_SCLK_PIN;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);
    if (err) {
        ESP_LOGE(TAG, "Failed to Initialize ADC\n");
    }

    ESP_LOGI(TAG, "Initialized SPI bus\n");

    ADS1120 ads1120;

    start_adc(ads1120);

    uint32_t data_index = 0;

    mqtt_message_t mqtt_message;
    mqtt_message_t dis_message;

    mqtt_message.start_micro = curr_time.pi_time_micro - curr_time.esp_time_micro + (uint64_t)esp_timer_get_time();
    uint64_t esp_packet_start_micro = (uint64_t)esp_timer_get_time();

    esp_wifi_get_mac(WIFI_IF_STA, mqtt_message.mac_address);
    ESP_LOGI(TAG, "Mac Address: %02x:%02x:%02x:%02x:%02x:%02x\n", mqtt_message.mac_address[0],
             mqtt_message.mac_address[1], mqtt_message.mac_address[2], mqtt_message.mac_address[3],
             mqtt_message.mac_address[4], mqtt_message.mac_address[5]);

    uint32_t count = 0;

    while (true) {
        if (ads1120.isDataReady()) // && mqtt_can_send_received())
        {
            mqtt_data_t data_val;

            uint16_t val;

            err = ads1120.readADC(&val);
            if (err) {
                continue;
            }

            data_val.data_point      = val;
            data_val.data_diff_micro = (uint64_t)esp_timer_get_time() - esp_packet_start_micro;

            mqtt_message.data[data_index] = data_val;

            data_index++;
            count++;

            if (count % 500 == 0) {
                ESP_LOGI(TAG, "%u\n", val);
                esp_task_wdt_reset();
                vTaskDelay(1);
            }

            if (data_index == MQTT_MESSAGE_LENGTH) {
                if (uxQueueSpacesAvailable(data_queue) == 0) {
                    xQueueReceive(data_queue, &dis_message, (TickType_t)10);
                }

                xQueueSend(data_queue, &mqtt_message, (TickType_t)10);

                esp_packet_start_micro = (uint64_t)esp_timer_get_time();
                mqtt_message.start_micro =
                    curr_time.pi_time_micro - curr_time.esp_time_micro + (uint64_t)esp_timer_get_time();
                data_index = 0;
            }
        }
    }
}

extern "C" void app_main(void)
{
    vTaskDelay(3000);

    wifi_start();

    data_queue = xQueueCreate(RTOS_QUEUE_SIZE, sizeof(mqtt_message_t));
    if (data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to initialize data queue\n");
    }

    xTaskCreate(read_adc_task, "read_adc", 8192, NULL, 5, &read_adc_handle);

    xTaskCreate(send_mqtt_task, "send_mqtt", 8192, NULL, 5, &send_mqtt_handle);
}

esp_err_t start_adc(ADS1120& adc)
{

    esp_err_t err = adc.init((gpio_num_t)ADC_CS_PIN, (gpio_num_t)ADC_DRDY_PIN, SPI2_HOST);
    if (err) {
        ESP_LOGE(TAG, "Failed to Initialize ADC\n");
        return err;
    }

    ESP_LOGI(TAG, "Initialized ADC\n");

    err = adc.setGain(1);
    if (err) {
        ESP_LOGE(TAG, "Failed to Set ADC Gain\n");
        return err;
    }

    err = adc.setVoltageRef(1);
    if (err) {
        ESP_LOGE(TAG, "Failed to Set ADC Gain\n");
        return err;
    }

    ESP_LOGI(TAG, "Set ADC Gain\n");

    adc.setDataRate(0x06);
    if (err) {
        ESP_LOGE(TAG, "Failed to Set ADC Data Rate\n");
        return err;
    }

    ESP_LOGI(TAG, "Set ADC Data Rate\n");

    adc.setOpMode(2);
    if (err) {
        ESP_LOGE(TAG, "Failed to Set ADC Op Mode\n");
        return err;
    }

    ESP_LOGI(TAG, "Set ADC OP Mode\n");

    adc.setConversionMode(1);
    if (err) {
        ESP_LOGE(TAG, "Failed to Set ADC Conversion Mode\n");
        return err;
    }

    ESP_LOGI(TAG, "Set ADC Conversion Mode\n");

    adc.setDRDYmode(1);
    if (err) {
        ESP_LOGE(TAG, "Failed to Set ADC DRDY Mode\n");
        return err;
    }

    ESP_LOGI(TAG, "Set ADC DRDY Mode\n");

    adc.setMultiplexer(0x09);
    if (err) {
        ESP_LOGE(TAG, "Failed to Set ADC Multiplexer\n");
        return err;
    }

    ESP_LOGI(TAG, "Set ADC Multiplexer\n");

    return ESP_OK;
}