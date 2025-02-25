#pragma once

#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <MQTTClient.h>
#include <chrono>
#include <ctime>
#include <thread>
#include <ostream>
#include <iostream>

namespace mqtt_client_publish_driver {

class MQTTClientPublishDriver : public rclcpp::Node {
public:
    MQTTClientPublishDriver();
    ~MQTTClientPublishDriver();

private:
    MQTTClient client_;
    std::shared_ptr<rclcpp::TimerBase> timer;
    void connect_to_broker();
    void publish_message_callback();
    void disconnect_from_broker();
};

}// namespace mqtt_client_publish_driver