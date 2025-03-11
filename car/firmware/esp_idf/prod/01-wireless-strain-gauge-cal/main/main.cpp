#include "driver/gpio.h"
#include <assert.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <mqttManager.hpp>
#include <sensorSetup.hpp>
#include <stdio.h>
#include <test.hpp>

#include <pb_decode.h>
#include <pb_encode.h>
#include <wsg_cal_com.pb.h>
#include <wsg_cal_data.pb.h>

#define SPI2_MOSI_PIN 18
#define SPI2_MISO_PIN 20
#define SPI2_SCLK_PIN 19

#define BROKER_URI "mqtt://10.42.0.1"
#define WIFI_SSID  "bjr_wireless_axle_host"
#define WIFI_PSWD  "bluejayracing"

#define WSG_ESP_PI_TOPIC   "esp/wsg_cal_esp"
#define WSG_PI_ESP_TOPIC   "esp/wsg_cal_pi"
#define WSG_CAL_DATA_TOPIC "esp/wsg_cal_data"

static const char* TAG = "main";

sensorSetup setup;
bool proto_status;
uint8_t proto_o_buf[300];
mqttManager* mqtt_manager;
mqtt_client_t* mqtt_client;

esp_err_t setUpSensor(void);

// Main Control Loop (For Production Firmware)
void vTaskMainControl(void*)
{
    pb_ostream_t o_stream = pb_ostream_from_buffer(proto_o_buf, sizeof(proto_o_buf));

    esp_err_t ret = setUpSensor();
    if (ret) {
        ESP_LOGE(TAG, "Failed to Set Up Sensor");
        return;
    }

    ESP_LOGI(TAG, "Configured Sensor Setup");

    mqtt_manager = mqttManager::getInstance();

    ret = mqtt_manager->init();
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize MQTT Manager: %d", ret);
        return;
    }

    mqtt_client = mqtt_manager->createClient(BROKER_URI);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        return;
    }

    // Connect to WiFi if not connected
    while (!mqtt_manager->isWiFiConnected()) {
        ret = mqtt_manager->connectWiFi(WIFI_SSID, WIFI_PSWD);
        if (ret) {
            ESP_LOGE(TAG, "Failed to Start WiFi Connection");
        }

        vTaskDelay(400);
    }

    // Connect to MQTT and Subscribe to Command Topic if not Connected
    while (!mqtt_manager->isClientConnected(mqtt_client)) {
        ret = mqtt_manager->clientConnect(mqtt_client);
        if (ret) {
            ESP_LOGE(TAG, "Failed to connect MQTT Client");
            vTaskDelay(400);
            continue;
        }

        vTaskDelay(200);
    }

    mqtt_manager->clientSubscribe(mqtt_client, WSG_PI_ESP_TOPIC, 2);

    for (;;) {
        cal_command_t poll_command;
        poll_command.command = 0;

        proto_status = pb_encode(&o_stream, cal_command_t_fields, &poll_command);
        if (!proto_status) {
            ESP_LOGE(TAG, "Failed to encode data message");
            vTaskDelay(100);
            continue;
        }

        ret = mqtt_manager->clientEnqueue(mqtt_client, proto_o_buf, o_stream.bytes_written, WSG_ESP_PI_TOPIC, 2);
        
        mqtt_message zero_message;
        esp_err_t ret = mqtt_manager->clientReceive(mqtt_client, zero_message, 100);
        if (ret) {
            ESP_LOGE(TAG, "Did not recieve Zero message");
            vTaskDelay(100);
            continue;
        }

        // TODO: Finish Recieving Command
    }

    // Zeroing WSG
    ret = setup.zero();
    if (ret) { // ZEROING FAILED
        ESP_LOGE(TAG, "Zeroing Failed");
        cal_command_t failed_zero_command;
        failed_zero_command.command = 1;

        proto_status = pb_encode(&o_stream, cal_command_t_fields, &failed_zero_command);
        if (!proto_status) {
            ESP_LOGE(TAG, "Failed to encode data message");
        }

        mqtt_manager->clientEnqueue(mqtt_client, proto_o_buf, o_stream.bytes_written, WSG_ESP_PI_TOPIC, 2);
    } else {   // ZEROING SUCCEEDED
        ESP_LOGI(TAG, "Zeroing Succeeded");
        cal_command_t success_zero_command;
        success_zero_command.command = 2;

        proto_status = pb_encode(&o_stream, cal_command_t_fields, &success_zero_command);
        if (!proto_status) {
            ESP_LOGE(TAG, "Failed to encode data message");
        }

        mqtt_manager->clientEnqueue(mqtt_client, proto_o_buf, o_stream.bytes_written, WSG_ESP_PI_TOPIC, 2);
    }

    ESP_LOGI(TAG, "Measuring and sending data");
    while (true) {
        // Connect to WiFi if not connected
        while (!mqtt_manager->isWiFiConnected()) {
            ret = mqtt_manager->connectWiFi(WIFI_SSID, WIFI_PSWD);
            if (ret) {
                ESP_LOGE(TAG, "Failed to Start WiFi Connection");
            }

            vTaskDelay(400);
        }

        // Connect to MQTT and Subscribe to Command Topic if not Connected
        while (!mqtt_manager->isClientConnected(mqtt_client)) {
            ret = mqtt_manager->clientConnect(mqtt_client);
            if (ret) {
                ESP_LOGE(TAG, "Failed to connect MQTT Client");
                vTaskDelay(400);
                continue;
            }

            vTaskDelay(200);
        }

        mqtt_message_t mqtt_message;
        cal_data_t measurements = cal_data_t_init_zero;
        sensor_measurement_t meas;

        for (int i = 0; i < 40; i++) {
            ret = setup.measure(&meas);

            measurements.adc_value[i] = meas.adc_value;
            measurements.dac_bias[i]  = meas.dac_bias;
            measurements.voltage[i]   = meas.voltage;
        }

        proto_status = pb_encode(&o_stream, cal_data_t_fields, &measurements);
        if (!proto_status) {
            ESP_LOGE(TAG, "Failed to encode data message");
        }

        mqtt_manager->clientEnqueue(mqtt_client, proto_o_buf, o_stream.bytes_written, WSG_CAL_DATA_TOPIC, 2);
    }
}

extern "C" void app_main(void)
{
    xTaskCreate(vTaskMainControl, "Main Control Loop", (1 << 14), NULL, 3, NULL);
    // Test test(ESP_LOG_DEBUG);
    // test.testSensorSetup();
}

esp_err_t setUpSensor(void)
{
    spi_bus_config_t spi_cfg;
    memset(&spi_cfg, 0, sizeof(spi_bus_config_t));

    spi_cfg.mosi_io_num   = SPI2_MOSI_PIN;
    spi_cfg.miso_io_num   = SPI2_MISO_PIN;
    spi_cfg.sclk_io_num   = SPI2_SCLK_PIN;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %d", ret);
        return ret;
    }

    gpio_config_t config;
    config.mode         = GPIO_MODE_OUTPUT;
    config.intr_type    = GPIO_INTR_DISABLE;
    config.pin_bit_mask = 1ULL << GPIO_NUM_17;
    config.pull_up_en   = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;

    ret = gpio_config(&config);
    if (ret) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %d", ret);
        return ret;
    }

    ad5626_init_param_t dac_params;
    dac_params.cs_pin   = GPIO_NUM_0;
    dac_params.ldac_pin = GPIO_NUM_17;
    dac_params.clr_pin  = GPIO_NUM_NC;
    dac_params.spi_host = SPI2_HOST;

    // Initialize the ADC instance
    ads1120_init_param_t adc_params;
    adc_params.cs_pin   = GPIO_NUM_21;
    adc_params.drdy_pin = GPIO_NUM_2;
    adc_params.spi_host = SPI2_HOST;

    ret = setup.init(adc_params, dac_params);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}