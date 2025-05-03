// wsg_drive_data_driver.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <baja_msgs/msg/data_chunk.hpp>
#include "MQTTClient.h"
#include <nlohmann/json.hpp>

namespace wsg_drive_data_driver {

using json = nlohmann::json;

class WSGDriveDataDriver : public rclcpp::Node {
public:
    WSGDriveDataDriver();
    ~WSGDriveDataDriver();

private: 
    void connect_to_broker();
    void subscribe_to_topics();
    void unsubscribe_from_topics();
    void disconnect_from_broker();
    static int message_arrived(void *context, char *topicName, int topicLen, MQTTClient_message *message);
    static void delivery_complete(void *context, MQTTClient_deliveryToken dt);
    static void connection_lost(void *context, char *cause);
    static int find_global_channel_id(const json& configJson, const std::string& macAddress,int localChannelId);
    static bool get_esp_calibration(const json& car_config_, const std::string& mac_to_find, int& zeroing_dac_value, int& strain_volt_slope);
    std::string make_utc_filename(void);

private:
    rclcpp::Publisher<baja_msgs::msg::DataChunk>::SharedPtr publisher_;
    std::string data_file_path;
    MQTTClient client_;
    json car_config_;
};

} // namespace wsg_drive_data_driver