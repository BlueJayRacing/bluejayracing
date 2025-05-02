#include "driver/gpio.h"
#include <calSensorSetup.hpp>
#include <config.hpp>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <hardEncoder.hpp>
#include <mqttManager.hpp>
#include <test.hpp>

#include <pb_decode.h>
#include <pb_encode.h>
#include <wsg_com.pb.h>
#include <wsg_cal_data.pb.h>
#include <wsg_drive_data.pb.h>
#include <wsg_pi_time.pb.h>

#define SPI2_MOSI_PIN 18
#define SPI2_MISO_PIN 20
#define SPI2_SCLK_PIN 19

#define BROKER_URI "mqtt://10.42.0.1"
#define WIFI_SSID  "bjr_wireless_axle_host"
#define WIFI_PSWD  "bluejayracing"

#define DEC_ESP_PI_TOPIC  "esp/wsg_dec_esp"
#define DEC_PI_ESP_TOPIC  "esp/wsg_dec_pi"
#define CAL_DATA_TOPIC    "esp/wsg_cal_data"
#define DRIVE_DATA_TOPIC  "esp/wsg_drive_data"
#define WSG_PI_TIME_TOPIC "esp/pi_time"

static const char* TAG = "main";

mqttManager* mqtt_manager;
QueueHandle_t drive_data_queue;
QueueHandle_t time_queue;
int pi_dac_bias = -1;

typedef struct ts_translation {
    uint64_t pi_time_us;
    uint32_t esp_time_us;
} ts_translation_t;

esp_err_t setUpCalSensor(calSensorSetup& setup);
esp_err_t setUpDriveSensor(driveSensorSetup& setup);
void DecisionTask(void);
void vTaskCalTask(void*);
void vTaskDriveRecordADCTask(void*);
void vTaskDriveSendDataTask(void*);

extern "C" void app_main(void)
{
#if ENABLE_TESTS == 1
    Test test(ESP_LOG_DEBUG);
    test.testADCDACEndtoEnd();
#else
    esp_log_level_set("mqttManager", ESP_LOG_NONE);
    DecisionTask();
#endif
}

void DecisionTask(void)
{
    mqtt_manager = mqttManager::getInstance();
    mqtt_client_t* main_mqtt_client;
    bool proto_status;

    esp_err_t ret = mqtt_manager->init(WIFI_SSID, WIFI_PSWD);
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize MQTT Manager: %d", ret);
        return;
    }

    main_mqtt_client = mqtt_manager->createClient(BROKER_URI);
    if (main_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        return;
    }

    wsg_com_poll_t wsg_poll;
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(wsg_poll.mac_address, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "MAC Address: %s", wsg_poll.mac_address);

    uint8_t proto_o_buf[30];
    pb_ostream_t o_stream = pb_ostream_from_buffer(proto_o_buf, sizeof(proto_o_buf));
    pb_encode(&o_stream, wsg_com_poll_t_fields, &wsg_poll);

    for (;;) {
        int num_wifi_disconnects = 0;

        // Connect to WiFi if not connected
        while (!mqtt_manager->isWiFiConnected()) {
            ESP_LOGE(TAG, "WiFi Is Not Connected");
            ret = mqtt_manager->connectWiFi();
            if (ret) {
                ESP_LOGE(TAG, "Failed to Start WiFi Connection");
            }

            mqtt_manager->waitWiFiConnect(40000 + 5000 * num_wifi_disconnects);
            num_wifi_disconnects++;
        }

        // Connect to MQTT and Subscribe to Command Topic if not Connected
        if (!mqtt_manager->isClientConnected(main_mqtt_client)) {
            bool mqtt_successfully_connected = false;

            for (int i = 0; i < 5; i++) {
                mqtt_manager->clientConnect(main_mqtt_client);

                vTaskDelay(1000);

                if (mqtt_manager->isClientConnected(main_mqtt_client)) {
                    mqtt_manager->clientSubscribe(main_mqtt_client, DEC_PI_ESP_TOPIC, 2);
                    mqtt_successfully_connected = true;
                    break;
                }
            }

            if (!mqtt_successfully_connected) {
                mqtt_manager->destroyClient(main_mqtt_client);
                main_mqtt_client = mqtt_manager->createClient(BROKER_URI);
                continue;
            }
        }

        ret = mqtt_manager->clientPublish(main_mqtt_client, proto_o_buf, o_stream.bytes_written, DEC_ESP_PI_TOPIC, 2);
        if (ret) {
            ESP_LOGE(TAG, "Failed to enqueue MQTT message: %d", ret);
            vTaskDelay(1000);
            continue;
        }

        mqtt_message start_message;
        ret = mqtt_manager->clientReceive(main_mqtt_client, start_message, 100);
        if (ret) {
            ESP_LOGE(TAG, "Did not recieve Start message");
            vTaskDelay(1000);
            continue;
        }

        pb_istream_t i_stream = pb_istream_from_buffer(start_message.payload.data(), start_message.payload_len);
        wsg_com_response_t pi_response;
        proto_status = pb_decode(&i_stream, wsg_com_response_t_fields, &pi_response);
        if (!proto_status) {
            ESP_LOGE(TAG, "Failed to decode Start message");
            vTaskDelay(1000);
            continue;
        }

        if (strcmp(wsg_poll.mac_address, pi_response.mac_address) != 0) {
            continue;
        }

        if (pi_response.command == 1) {
            ESP_LOGI(TAG, "Executing Calibration Task");
            xTaskCreate(vTaskCalTask, "Calibration Main Control Loop", (1 << 15), NULL, 3, NULL);
            break;
        } else if (pi_response.command == 2) {
            ESP_LOGI(TAG, "Executing Drive Task");
            drive_data_queue = xQueueCreate(20, sizeof(wsg_drive_data_t));
            time_queue       = xQueueCreate(10, sizeof(ts_translation_t));
            pi_dac_bias = pi_response.dac_bias;
            ESP_LOGI(TAG, "Received DAC Bias Value: %d", pi_dac_bias);

            xTaskCreate(vTaskDriveRecordADCTask, "Drive Record ADC Task", 24000, NULL, 3, NULL);
            xTaskCreate(vTaskDriveSendDataTask, "Drive Send Data Task", 24000, NULL, 3, NULL);
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
        int num_wifi_disconnects = 0;

        // Connect to WiFi if not connected
        while (!mqtt_manager->isWiFiConnected()) {
            ESP_LOGE(TAG, "WiFi Is Not Connected");
            ret = mqtt_manager->connectWiFi();
            if (ret) {
                ESP_LOGE(TAG, "Failed to Start WiFi Connection");
            }

            mqtt_manager->waitWiFiConnect(40000 + 5000 * num_wifi_disconnects);
            num_wifi_disconnects++;
        }

        // Connect to MQTT and Subscribe to Command Topic if not Connected
        if (!mqtt_manager->isClientConnected(cal_mqtt_client)) {
            bool mqtt_successfully_connected = false;

            for (int i = 0; i < 5; i++) {
                mqtt_manager->clientConnect(cal_mqtt_client);

                vTaskDelay(1000);

                if (mqtt_manager->isClientConnected(cal_mqtt_client)) {
                    mqtt_manager->clientSubscribe(cal_mqtt_client, WSG_PI_TIME_TOPIC, 2);
                    mqtt_successfully_connected = true;
                    break;
                }
            }

            if (!mqtt_successfully_connected) {
                mqtt_manager->destroyClient(cal_mqtt_client);
                cal_mqtt_client = mqtt_manager->createClient(BROKER_URI);
                continue;
            }
        }

        wsg_cal_data_t measurements = wsg_cal_data_t_init_zero;
        cal_measurement_t meas;

        for (int i = 0; i < 40; i++) {
            ret = cal_setup.measure(&meas);

            measurements.adc_value[i] = meas.adc_value;
            measurements.dac_bias[i]  = meas.dac_bias;
            measurements.voltage[i]   = meas.voltage;
        }

        uint8_t proto_o_buf[800];
        pb_ostream_t o_stream = pb_ostream_from_buffer(proto_o_buf, sizeof(proto_o_buf));

        proto_status = pb_encode(&o_stream, wsg_cal_data_t_fields, &measurements);
        if (!proto_status) {
            ESP_LOGE(TAG, "Failed to encode data message: %s\n", PB_GET_ERROR(&o_stream));
        }

        mqtt_manager->clientPublish(cal_mqtt_client, proto_o_buf, o_stream.bytes_written, CAL_DATA_TOPIC, 2);

        vTaskDelay(1); // 1 millisecond
    }
}

void vTaskDriveRecordADCTask(void*)
{
    driveSensorSetup drive_setup;

    esp_err_t ret = setUpDriveSensor(drive_setup);
    if (ret) {
        ESP_LOGE(TAG, "Failed to Set Up Sensor");
        return;
    }

    ESP_LOGI(TAG, "Configured Sensor Setup");

    // Set DAC Value
    drive_setup.setDACValue(pi_dac_bias);

    ESP_LOGI(TAG, "Measuring and sending data");
    wsg_drive_data_t measurements = wsg_drive_data_t_init_zero;

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(measurements.mac_address, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "MAC Address: %s", measurements.mac_address);

    ts_translation_t time_trans = {0, 0};
    xQueueReceive(time_queue, &time_trans, portMAX_DELAY);
    // ESP_LOGI(TAG, "Received Time Translation: pi_time_us: %lld, esp_time_us: %ld", time_trans.pi_time_us,
    //          time_trans.esp_time_us);

    drive_cfg_t sample_cfg  = {drive_cfg_t::MEASURING_MODE, drive_cfg_t::STRAIN_GAUGE};
    drive_cfg_t excitn_cfg  = {drive_cfg_t::MEASURING_MODE, drive_cfg_t::EXCITATION};
    drive_cfg_t dacbias_cfg = {drive_cfg_t::MEASURING_MODE, drive_cfg_t::DAC_BIAS};

    while (true) {
        drive_measurement_t meas;

        if (uxQueueMessagesWaiting(time_queue) > 0) {
            xQueueReceive(time_queue, &time_trans, 0);
            // ESP_LOGI(TAG, "Received Time Translation: pi_time_us: %lld, esp_time_us: %ld", time_trans.pi_time_us,
            //          time_trans.esp_time_us);
        }

        uint32_t packet_start_time_us = esp_timer_get_time();
        measurements.base_timestamp   = time_trans.pi_time_us + packet_start_time_us - time_trans.esp_time_us;

        for (int i = 0; i < NUM_SAMPLES_PER_MESSAGE; i++) {
            drive_setup.configure(sample_cfg);
            drive_setup.measure(true, &meas);

            measurements.timestamp_deltas[i] = esp_timer_get_time() - packet_start_time_us;
            measurements.values[i]           = meas.voltage;
        }

        drive_setup.configure(dacbias_cfg);
        drive_setup.measure(true, &meas);
        measurements.dac_bias = meas.voltage;

        drive_setup.configure(excitn_cfg);
        drive_setup.measure(true, &meas);
        measurements.excitation_voltage = meas.voltage;

        if (uxQueueSpacesAvailable(drive_data_queue) > 0) {
            xQueueSend(drive_data_queue, &measurements, 0);
        }
    }
}

void vTaskDriveSendDataTask(void*)
{
    bool proto_status;
    esp_err_t ret;
    mqtt_client_t* drive_mqtt_client;
    std::array<uint8_t, 12000> proto_o_buf;

    drive_mqtt_client = mqtt_manager->createClient(BROKER_URI);
    if (drive_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        return;
    }

    mqtt_manager->clientSubscribe(drive_mqtt_client, WSG_PI_TIME_TOPIC, 2);

    while (true) {
        int num_wifi_disconnects = 0;

        // Connect to WiFi if not connected
        while (!mqtt_manager->isWiFiConnected()) {
            ESP_LOGE(TAG, "WiFi Is Not Connected");
            ret = mqtt_manager->connectWiFi();
            if (ret) {
                ESP_LOGE(TAG, "Failed to Start WiFi Connection");
            }

            mqtt_manager->waitWiFiConnect(40000 + 5000 * num_wifi_disconnects);
            num_wifi_disconnects++;
        }

        // Connect to MQTT and Subscribe to Command Topic if not Connected
        if (!mqtt_manager->isClientConnected(drive_mqtt_client)) {
            bool mqtt_successfully_connected = false;

            for (int i = 0; i < 5; i++) {
                mqtt_manager->clientConnect(drive_mqtt_client);

                vTaskDelay(1000);

                if (mqtt_manager->isClientConnected(drive_mqtt_client)) {
                    mqtt_manager->clientSubscribe(drive_mqtt_client, WSG_PI_TIME_TOPIC, 2);
                    mqtt_successfully_connected = true;
                    break;
                }
            }

            if (!mqtt_successfully_connected) {
                mqtt_manager->destroyClient(drive_mqtt_client);
                drive_mqtt_client = mqtt_manager->createClient(BROKER_URI);
                continue;
            }
        }

        mqtt_message time_message;

        if (mqtt_manager->clientReceive(drive_mqtt_client, time_message, 2) == ESP_OK) {
            pb_istream_t i_stream = pb_istream_from_buffer(time_message.payload.data(), time_message.payload_len);
            wsg_pi_time_t updated_time;
            proto_status = pb_decode(&i_stream, wsg_pi_time_t_fields, &updated_time);
            if (!proto_status) {
                ESP_LOGE(TAG, "Failed to decode Start message");
                vTaskDelay(1000);
                continue;
            }

            ts_translation_t new_translation = {updated_time.time_us, time_message.esp_time};
            xQueueSend(time_queue, &new_translation, 0);
        }

        wsg_drive_data_t measurements = wsg_drive_data_t_init_zero;

        if (xQueueReceive(drive_data_queue, &measurements, 1000) != pdTRUE) {
            continue;
        }

        int bytes_written = hardEncoder::encodeDriveData(measurements, proto_o_buf);

        mqtt_manager->clientPublish(drive_mqtt_client, proto_o_buf.data(), bytes_written, DRIVE_DATA_TOPIC, 2);
        ESP_LOGI(TAG, "Sent Data: %lld", esp_timer_get_time() / 1000);
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