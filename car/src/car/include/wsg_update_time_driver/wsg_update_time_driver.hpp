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

namespace wsg_update_time_driver {

class WSGUpdateTimeDriver : public rclcpp::Node {
public:
    WSGUpdateTimeDriver();
    ~WSGUpdateTimeDriver();

private:
    MQTTClient client_;
    std::shared_ptr<rclcpp::TimerBase> timer;
    void connect_to_broker();
    void publish_message_callback();
    void disconnect_from_broker();
};

}// namespace wsg_update_time_driver