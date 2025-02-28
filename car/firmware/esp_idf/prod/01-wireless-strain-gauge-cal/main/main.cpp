#include "driver/gpio.h"
#include <assert.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <sensorSetup.hpp>
#include <stdio.h>
#include <test.hpp>
#include <mqttManager.hpp>

#include <wsg_cal_data.pb.h>
#include <wsg_cal_com.pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

#define SPI2_MOSI_PIN 18
#define SPI2_MISO_PIN 20
#define SPI2_SCLK_PIN 19

#define BROKER_URI  "mqtt://10.42.0.1"
#define WIFI_SSID   "bjr_wireless_axle_host"
#define WIFI_PSWD   "bluejayracing"

#define START_CAL_TOPIC "esp/start_cal"
#define CAL_DATA_TOPIC  "esp/cal_data"

static const char* TAG = "main";

typedef enum device_mode {
    WAITING_MODE,
    ZEROING_MODE,
    MEASURING_MODE,
} device_mode_t;

sensorSetup setup;
device_mode_t device_mode = WAITING_MODE;
bool proto_status;
uint8_t proto_i_buf[30];
uint8_t proto_o_buf[100];

esp_err_t setUpSensor(void);

// Main Control Loop
void vTaskMainControl(void*) {
    esp_err_t ret = setUpSensor();
    if (ret) {
        ESP_LOGE(TAG, "Failed to Set Up Sensor");
        return;
    }

    ESP_LOGI(TAG, "Configured Sensor Setup");

    mqttManager* mqtt_manager = mqttManager::getInstance();

    ret = mqtt_manager->init();
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize MQTT Manager: %d", ret);
        return;
    }

    mqtt_client_t* mqtt_client = mqtt_manager->createClient(BROKER_URI);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        return;
    }

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

            ret = mqtt_manager->clientSubscribe(mqtt_client, START_CAL_TOPIC, 2);
            if (ret) {
                ESP_LOGE(TAG, "Failed to subscribe to Start Calibration Command Topic");
                vTaskDelay(100);
                continue;
            }
        }

        mqtt_message_t mqtt_message;

        ret = mqtt_manager->clientReceive(mqtt_client, mqtt_message, 1);
        if (!ret) {
            cal_command_t cal_command;

            pb_istream_t i_stream = pb_istream_from_buffer(mqtt_message.payload.data(), mqtt_message.payload_len);

            proto_status = pb_decode(&i_stream, cal_command_t_fields, &cal_command);

            if (!proto_status) {
                ESP_LOGE(TAG, "Failed decoding recieved proto message");
            } else {
                switch (cal_command.command) {
                    case 1:
                        ESP_LOGI(TAG, "Received Data Record Command");
                        break;
                    case 2:
                        ESP_LOGI(TAG, "Recieved Data Stop Command");
                        break;
                    case 3:
                        ESP_LOGI(TAG, "Recieved Data Zero Command");
                        break;
                }
            }
        }

        switch (device_mode) {
            case WAITING_MODE:
                ESP_LOGI(TAG, "Waiting For Command");
                vTaskDelay(100);
                break;
            case ZEROING_MODE:
                ESP_LOGI(TAG, "Zeroing");
                ret = setup.zero();
                if (ret) {
                    ESP_LOGE(TAG, "Failed Zeroing");
                } else {
                    ESP_LOGI(TAG, "Successfully Zeroed");
                }

                ESP_LOGI(TAG, "Returning to Waiting Mode");
                device_mode = WAITING_MODE;
                break;
            case MEASURING_MODE:
                ESP_LOGI(TAG, "Measuring and sending data");

                cal_data_t cal_data;
                break;
        }
    }
}

extern "C" void app_main(void)
{
    xTaskCreate(vTaskMainControl, "Main Control Loop", (1 << 14), NULL , 3, NULL);
    // Test test(ESP_LOG_DEBUG);
    // test.testSensorSetup();
} 

esp_err_t setUpSensor(void) {
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
    config.mode = GPIO_MODE_OUTPUT;
    config.intr_type = GPIO_INTR_DISABLE;
    config.pin_bit_mask = 1ULL << GPIO_NUM_17;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
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