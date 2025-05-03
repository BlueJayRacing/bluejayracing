#include <chrono>
#include <cstring>
#include <fstream>
#include <limits>
#include <format>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "pb_decode.h"
#include "pb_encode.h"
#include "wsg_com.pb.h"
#include "wsg_drive_data.pb.h"
#include <wsg_drive_data_driver/wsg_drive_data_driver.hpp>

#define LOCALHOST_ADDRESS    "localhost:1883"
#define CLIENTID             "wsg_drive_data_node"
#define WSG_ESP_PI_TOPIC     "esp/wsg_dec_esp"
#define WSG_PI_ESP_TOPIC     "esp/wsg_dec_pi"
#define WSG_DRIVE_DATA_TOPIC "esp/wsg_drive_data"
#define QOS                  2
#define TIMEOUT              10000L

namespace wsg_drive_data_driver
{

using json = nlohmann::json;

WSGDriveDataDriver::WSGDriveDataDriver() : Node("wsg_drive_data_driver")
{
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publisher_                 = create_publisher<baja_msgs::msg::DataChunk>(
        "mqtt_data", rclcpp::QoS(1000).best_effort().durability_volatile(), pub_options);
    connect_to_broker();
    subscribe_to_topics();

    data_file_path = make_utc_filename();

    std::ofstream csvFile(data_file_path, std::ios::app);
    csvFile.close();

    // Get the path to the package
    std::string car_cfg_pkg_path  = ament_index_cpp::get_package_share_directory("car_config");
    std::string car_cfg_json_path = car_cfg_pkg_path + "/config/car_config.json";

    // Open and read JSON file
    std::ifstream car_cfg_json(car_cfg_json_path);
    if (!car_cfg_json.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open JSON file: %s", car_cfg_json_path.c_str());
        return;
    }

    car_config_ = json::parse(car_cfg_json);
}

WSGDriveDataDriver::~WSGDriveDataDriver()
{
    unsubscribe_from_topics();
    disconnect_from_broker();
    MQTTClient_destroy(&client_);
}

void WSGDriveDataDriver::connect_to_broker()
{
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;

    if ((rc = MQTTClient_create(&client_, LOCALHOST_ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL)) !=
        MQTTCLIENT_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create client, return code %d", rc);
        rclcpp::shutdown();
        return;
    }

    if ((rc = MQTTClient_setCallbacks(client_, this, connection_lost, message_arrived, delivery_complete)) !=
        MQTTCLIENT_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set callbacks, return code %d", rc);
        rclcpp::shutdown();
        return;
    }

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession      = 1;
    if ((rc = MQTTClient_connect(client_, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to connect, return code %d", rc);
        rclcpp::shutdown();
        return;
    }
}

void WSGDriveDataDriver::subscribe_to_topics()
{
    int rc;
    if ((rc = MQTTClient_subscribe(client_, WSG_DRIVE_DATA_TOPIC, QOS)) != MQTTCLIENT_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to subscribe to calibration data topic, return code %d", rc);
        rclcpp::shutdown();
    }

    if ((rc = MQTTClient_subscribe(client_, WSG_ESP_PI_TOPIC, QOS)) != MQTTCLIENT_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to subscribe to (ESP - PI) topic, return code %d", rc);
        rclcpp::shutdown();
    }
}

void WSGDriveDataDriver::unsubscribe_from_topics()
{
    int rc;
    if ((rc = MQTTClient_unsubscribe(client_, WSG_DRIVE_DATA_TOPIC)) != MQTTCLIENT_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to unsubscribe, return code %d", rc);
    }

    if ((rc = MQTTClient_unsubscribe(client_, WSG_ESP_PI_TOPIC)) != MQTTCLIENT_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to unsubscribe, return code %d", rc);
    }
}

void WSGDriveDataDriver::disconnect_from_broker()
{
    int rc;
    if ((rc = MQTTClient_disconnect(client_, 10000)) != MQTTCLIENT_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to disconnect, return code %d", rc);
    }
}

int WSGDriveDataDriver::message_arrived(void* context, char* topicName, int topicLen, MQTTClient_message* message)
{
    WSGDriveDataDriver* driver = static_cast<WSGDriveDataDriver*>(context);

    MQTTClient_deliveryToken token;
    MQTTClient_message echo_message = MQTTClient_message_initializer;

    std::string topic_str(topicName);

    do {
        if (topic_str == WSG_ESP_PI_TOPIC) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received Message from ESP32 Waiting");
            wsg_com_poll_t esp_poll = wsg_com_poll_t_init_zero;

            // Decode Polling Message
            {
                pb_istream_t istream = pb_istream_from_buffer((const pb_byte_t*)message->payload, message->payloadlen);

                if (!pb_decode(&istream, wsg_com_poll_t_fields, &esp_poll)) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error decoding ESP Command: %s",
                                 PB_GET_ERROR(&istream));
                    break;
                }

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received polling signal");
            }

            std::string poll_mac_address(esp_poll.mac_address);
            int zeroing_dac_value;
            int strain_volt_slope;

            get_esp_calibration(driver->car_config_, poll_mac_address, zeroing_dac_value, strain_volt_slope);

            // Send Start Message
            {
                wsg_com_response_t cmd = wsg_com_response_t_init_zero;
                strcpy(cmd.mac_address, esp_poll.mac_address);
                cmd.dac_bias = zeroing_dac_value;
                cmd.command  = 2;

                uint8_t buffer[128]  = {0};
                pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));

                if (!pb_encode(&ostream, wsg_com_response_t_fields, &cmd)) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error encoding wsg_com_response_t: %s",
                                 PB_GET_ERROR(&ostream));
                    break;
                }

                MQTTClient_deliveryToken token;
                MQTTClient_message start_message = MQTTClient_message_initializer;

                start_message.payload    = buffer;
                start_message.payloadlen = (int)ostream.bytes_written;
                start_message.qos        = 0;
                start_message.retained   = 0;

                MQTTClient_publishMessage(driver->client_, WSG_PI_ESP_TOPIC, &start_message, &token);

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "published drive start command");
            }
        } else if (topic_str == WSG_DRIVE_DATA_TOPIC) {
            wsg_drive_data_t drive_data = wsg_drive_data_t_init_zero;
            pb_istream_t istream = pb_istream_from_buffer((const pb_byte_t*)message->payload, message->payloadlen);

            if (!pb_decode(&istream, wsg_drive_data_t_fields, &drive_data)) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error decoding ESP Data: %s", PB_GET_ERROR(&istream));
                break;
            }

            std::string esp_mac_address(drive_data.mac_address);
            int zeroing_dac_value;
            int strain_volt_slope;

            get_esp_calibration(driver->car_config_, esp_mac_address, zeroing_dac_value, strain_volt_slope);

            int global_channel_id =
                find_global_channel_id(driver->car_config_, esp_mac_address, drive_data.sample_channel_id);

            std::ofstream csvFile(driver->data_file_path, std::ios::app);

            // Reuse AnalogChannel object
            baja_msgs::msg::UnifiedSample sample;
            sample.mac_address         = esp_mac_address;
            sample.ros_channel_id      = global_channel_id;
            sample.internal_channel_id = drive_data.sample_channel_id;

            // Needs to put time one to one
            baja_msgs::msg::DataChunk data_chunk;
            for (int i = 0; i < 1000; i++) {
                sample.recorded_time = drive_data.base_timestamp + drive_data.timestamp_deltas[i];
                sample.data_value    = static_cast<double>(drive_data.values[i]);
                data_chunk.samples.push_back(sample);

                if (csvFile.is_open()) {
                    csvFile << int(global_channel_id) << ","
                            << drive_data.base_timestamp + drive_data.timestamp_deltas[i] << "," << (drive_data.values[i] - 2.50) * strain_volt_slope
                            << ",\n";
                }
            }
            driver->publisher_->publish(data_chunk);

            csvFile.close(); // Close the file after writing
        }
    } while (false);

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);

    return 1;
}

void WSGDriveDataDriver::delivery_complete(void* context, MQTTClient_deliveryToken dt)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Message with token value %d delivery confirmed", dt);
}

void WSGDriveDataDriver::connection_lost(void* context, char* cause)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection lost");
    if (cause) RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "     cause: %s", cause);
}

int WSGDriveDataDriver::find_global_channel_id(const json& configJson, const std::string& macAddress,
                                               int localChannelId)
{
    // Convert the integer localChannelId to string for JSON lookup
    std::string localChannelIdStr = std::to_string(localChannelId);

    // Check if we have ESP32 configuration
    if (!configJson.contains("esp32") || !configJson["esp32"].contains("variants")) {
        return -1;
    }

    // Iterate through all ESP32 variants
    for (const auto& [variantName, variantConfig] : configJson["esp32"]["variants"].items()) {
        // Check if this variant has MAC addresses
        if (!variantConfig.contains("mac_addresses")) {
            continue;
        }

        // Check if the requested MAC address is in this variant's list
        const auto& macAddresses = variantConfig["mac_addresses"];
        bool macFound            = false;

        for (const auto& mac : macAddresses) {
            if (mac.get<std::string>() == macAddress) {
                macFound = true;
                break;
            }
        }

        if (macFound) {
            // Found the variant with matching MAC address
            // Check if it has the requested local channel ID
            if (variantConfig.contains("channel_mapping") &&
                variantConfig["channel_mapping"].contains(localChannelIdStr)) {
                // Return the global channel ID as an integer
                return variantConfig["channel_mapping"][localChannelIdStr].get<int>();
            }
            return -1; // MAC found but channel not found
        }
    }

    // MAC address not found in any variant
    return -1;
}

bool WSGDriveDataDriver::get_esp_calibration(const json& car_config_, const std::string& mac_to_find,
                                             int& zeroing_dac_value, int& strain_volt_slope)
{
    auto toLower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
        return s;
    };

    const auto mac_target = toLower(mac_to_find);

    // 1.  Make sure the tree has the expected hierarchy.
    if (!car_config_.contains("esp32") || !car_config_["esp32"].contains("variants")) return false;

    const json& variants = car_config_["esp32"]["variants"];

    // 2.  Iterate through the variant objects.
    for (const auto& [variant_name, variant] : variants.items()) {
        if (!variant.contains("mac_addresses")) continue;

        for (const auto& mac_json : variant["mac_addresses"]) {
            if (toLower(mac_json.get<std::string>()) == mac_target) {
                // 3.  Extract the calibration fields (default to zero if absent).
                zeroing_dac_value = variant.value("zeroing_dac_value", 0);
                strain_volt_slope = variant.value("strain_volt_slope", 0);
                return true;
            }
        }
    }
    return false; // not found
}

std::string WSGDriveDataDriver::make_utc_filename(void)
{
    using clock = std::chrono::system_clock;

    std::time_t now = clock::to_time_t(clock::now());
    std::tm     tm  = *std::gmtime(&now);   // convert to UTC

    std::ostringstream oss;
    oss << "src/bjr_packages/car/data_"                           // prefix
        << std::setw(2) << std::setfill('0') << tm.tm_mon + 1 << '_'  // month
        << std::setw(2) << tm.tm_mday        << '_'                    // day
        << std::setw(2) << tm.tm_hour        << ':'                    // hour
        << std::setw(2) << tm.tm_min         << ':'                    // minute
        << std::setw(2) << tm.tm_sec         << ".txt";                // second + ext

    return oss.str();
}

} // namespace wsg_drive_data_driver
