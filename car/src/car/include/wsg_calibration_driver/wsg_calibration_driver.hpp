// wsg_calibration_driver.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <baja_msgs/msg/observation.hpp>
#include "MQTTClient.h"

namespace wsg_calibration_driver {

class WSGCalibrationDriver : public rclcpp::Node {
public:
    WSGCalibrationDriver();
    ~WSGCalibrationDriver();

private: 
    void connect_to_broker();
    void subscribe_to_topics();
    void unsubscribe_from_topics();
    void disconnect_from_broker();
    static int message_arrived(void *context, char *topicName, int topicLen, MQTTClient_message *message);
    static void delivery_complete(void *context, MQTTClient_deliveryToken dt);
    static void connection_lost(void *context, char *cause);

    rclcpp::Publisher<baja_msgs::msg::Observation>::SharedPtr publisher_;
    MQTTClient client_;
};

} // namespace wsg_calibration_driver