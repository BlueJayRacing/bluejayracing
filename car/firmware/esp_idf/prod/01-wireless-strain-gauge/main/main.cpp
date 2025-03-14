#include "driver/gpio.h"
#include <assert.h>
#include <calSensorSetup.hpp>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <mqttManager.hpp>
#include <stdio.h>
#include <test.hpp>
#include <esp_timer.h>

#include <pb_decode.h>
#include <pb_encode.h>
#include <wsg_cal_com.pb.h>
#include <wsg_cal_data.pb.h>
#include <wsg_drive_data.pb.h>

#define SPI2_MOSI_PIN 18
#define SPI2_MISO_PIN 20
#define SPI2_SCLK_PIN 19

#define BROKER_URI "mqtt://10.42.0.1"
#define WIFI_SSID  "bjr_wireless_axle_host"
#define WIFI_PSWD  "bluejayracing"

#define DEC_ESP_PI_TOPIC "esp/wsg_dec_esp"
#define DEC_PI_ESP_TOPIC "esp/wsg_dec_pi"
#define CAL_DATA_TOPIC   "esp/wsg_cal_data"
#define DRIVE_DATA_TOPIC   "esp/wsg_drive_data"

static const char* TAG = "main";

mqttManager* mqtt_manager;

esp_err_t setUpCalSensor(calSensorSetup& setup);
esp_err_t setUpDriveSensor(driveSensorSetup& setup);
void DecisionTask(void);
void vTaskCalTask(void*);
void vTaskDriveTask(void*);

extern "C" void app_main(void)
{
    // Test test(ESP_LOG_DEBUG);
    // test.testDriveSensorSetup();
    DecisionTask();
}

void DecisionTask(void)
{
    mqtt_manager = mqttManager::getInstance();
    mqtt_client_t* main_mqtt_client;
    bool proto_status;

    esp_err_t ret = mqtt_manager->init();
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize MQTT Manager: %d", ret);
        return;
    }

    main_mqtt_client = mqtt_manager->createClient(BROKER_URI);
    if (main_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        return;
    }

    cal_command_t poll_command;
    poll_command.command = 1;

    for (;;) {
        // Connect to WiFi if not connected
        while (!mqtt_manager->isWiFiConnected()) {
            ret = mqtt_manager->connectWiFi(WIFI_SSID, WIFI_PSWD);
            if (ret) {
                ESP_LOGE(TAG, "Failed to Start WiFi Connection");
            }

            vTaskDelay(400);
        }

        // Connect to MQTT and Subscribe to Command Topic if not Connected
        while (!mqtt_manager->isClientConnected(main_mqtt_client)) {
            ret = mqtt_manager->clientConnect(main_mqtt_client);
            if (ret) {
                ESP_LOGE(TAG, "Failed to connect MQTT Client");
                vTaskDelay(400);
                continue;
            }

            vTaskDelay(200);
            mqtt_manager->clientSubscribe(main_mqtt_client, DEC_PI_ESP_TOPIC, 2);
        }

        uint8_t proto_o_buf[30];
        pb_ostream_t o_stream = pb_ostream_from_buffer(proto_o_buf, sizeof(proto_o_buf));
        pb_encode(&o_stream, cal_command_t_fields, &poll_command);

        ret = mqtt_manager->clientEnqueue(main_mqtt_client, proto_o_buf, o_stream.bytes_written, DEC_ESP_PI_TOPIC, 2);
        if (ret) {
            ESP_LOGE(TAG, "Failed to enqueue MQTT message: %d", ret);
            vTaskDelay(100);
            continue;
        }

        mqtt_message start_message;
        ret = mqtt_manager->clientReceive(main_mqtt_client, start_message, 100);
        if (ret) {
            ESP_LOGE(TAG, "Did not recieve Start message");
            vTaskDelay(100);
            continue;
        }

        pb_istream_t i_stream = pb_istream_from_buffer(start_message.payload.data(), start_message.payload_len);
        cal_command_t start_command;
        proto_status = pb_decode(&i_stream, cal_command_t_fields, &start_command);
        if (!proto_status) {
            ESP_LOGE(TAG, "Failed to decode Start message");
            vTaskDelay(100);
            continue;
        }

        if (start_command.command == 1) {
            ESP_LOGI(TAG, "Executing Calibration Main Task");
            xTaskCreate(vTaskCalTask, "Calibration Main Control Loop", (1 << 14), NULL, 3, NULL);
            break;
        } else if (start_command.command == 2) {
            ESP_LOGI(TAG, "Executing Drive Main Task");
            xTaskCreate(vTaskDriveTask, "Drive Main Control Loop", (1 << 15), NULL, 3, NULL);
            break;
        }
    }

    mqtt_manager->destroyClient(main_mqtt_client);
}

// Main Control Loop For Calibration
void vTaskCalTask(void*)
{
    calSensorSetup cal_setup;
    bool proto_status;
    mqtt_client_t* cal_mqtt_client;

    cal_mqtt_client = mqtt_manager->createClient(BROKER_URI);
    if (cal_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        return;
    }

    esp_err_t ret = setUpCalSensor(cal_setup);
    if (ret) {
        ESP_LOGE(TAG, "Failed to Set Up Sensor");
        return;
    }

    ESP_LOGI(TAG, "Configured Sensor Setup");

    // Zeroing WSG
    ret = cal_setup.zero();
    if (ret) { // ZEROING FAILED
        ESP_LOGE(TAG, "Zeroing Failed");
    } else { // ZEROING SUCCEEDED
        ESP_LOGI(TAG, "Zeroing Succeeded");
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
        while (!mqtt_manager->isClientConnected(cal_mqtt_client)) {
            ret = mqtt_manager->clientConnect(cal_mqtt_client);
            if (ret) {
                ESP_LOGE(TAG, "Failed to connect MQTT Client");
                vTaskDelay(400);
                continue;
            }

            vTaskDelay(200);
        }

        cal_data_t measurements = cal_data_t_init_zero;
        cal_measurement_t meas;

        for (int i = 0; i < 40; i++) {
            ret = cal_setup.measure(&meas);

            measurements.adc_value[i] = meas.adc_value;
            measurements.dac_bias[i]  = meas.dac_bias;
            measurements.voltage[i]   = meas.voltage;
        }

        uint8_t proto_o_buf[800];
        pb_ostream_t o_stream = pb_ostream_from_buffer(proto_o_buf, sizeof(proto_o_buf));

        proto_status = pb_encode(&o_stream, cal_data_t_fields, &measurements);
        if (!proto_status) {
            ESP_LOGE(TAG, "Failed to encode data message: %s\n", PB_GET_ERROR(&o_stream));
        }

        mqtt_manager->clientEnqueue(cal_mqtt_client, proto_o_buf, o_stream.bytes_written, CAL_DATA_TOPIC, 2);

        vTaskDelay(1); // 1 millisecond
    }
}

void vTaskDriveTask(void*)
{
    driveSensorSetup drive_setup;
    bool proto_status;
    mqtt_client_t* drive_mqtt_client;

    drive_mqtt_client = mqtt_manager->createClient(BROKER_URI);
    if (drive_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        return;
    }

    esp_err_t ret = setUpDriveSensor(drive_setup);
    if (ret) {
        ESP_LOGE(TAG, "Failed to Set Up Sensor");
        return;
    }

    ESP_LOGI(TAG, "Configured Sensor Setup");

    // Zeroing WSG
    ret = drive_setup.zero();
    if (ret) { // ZEROING FAILED
        ESP_LOGE(TAG, "Zeroing Failed");
    } else { // ZEROING SUCCEEDED
        ESP_LOGI(TAG, "Zeroing Succeeded");
    }

    ESP_LOGI(TAG, "Measuring and sending data");
    drive_data_t measurements;
    uint8_t proto_o_buf[16000];

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
        while (!mqtt_manager->isClientConnected(drive_mqtt_client)) {
            ret = mqtt_manager->clientConnect(drive_mqtt_client);
            if (ret) {
                ESP_LOGE(TAG, "Failed to connect MQTT Client");
                vTaskDelay(400);
                continue;
            }

            vTaskDelay(200);
        }

        drive_measurement_t meas;

        for (int i = 0; i < 1000; i++) {
            ret = drive_setup.measure(&meas, true);

            measurements.time[i] = esp_timer_get_time();
            measurements.voltage[i]   = meas.voltage;
        }

        pb_ostream_t o_stream = pb_ostream_from_buffer(proto_o_buf, sizeof(proto_o_buf));

        proto_status = pb_encode(&o_stream, drive_data_t_fields, &measurements);
        if (!proto_status) {
            ESP_LOGE(TAG, "Failed to encode data message: %s\n", PB_GET_ERROR(&o_stream));
        }

        mqtt_manager->clientEnqueue(drive_mqtt_client, proto_o_buf, o_stream.bytes_written, DRIVE_DATA_TOPIC, 2);

        vTaskDelay(1); // 1 millisecond
    }
}

esp_err_t setUpCalSensor(calSensorSetup& cal_setup)
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

    ret = cal_setup.init(adc_params, dac_params);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t setUpDriveSensor(driveSensorSetup& drive_setup)
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

    ret = drive_setup.init(adc_params, dac_params);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}