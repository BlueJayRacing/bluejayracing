#include "xbee_driver/xbee_driver.hpp"
#include <chrono>

#define XBEE_DRIVER_MAX_SEND_RETRIES 2

#define RF_RATE 2 // 0: 10 kb/s, 1: 110 kb/s, 2: 250 kb/s

#ifndef RF_RATE
    #error "RF_RATE must be defined and be 0, 1, or 2"
#endif
#if RF_RATE == 0
  #define POLLING_INTERVAL 120000 // useconds
  #define FULL_QUEUE_WAIT_TIME 480000 // useconds
#elif RF_RATE == 1
  #define POLLING_INTERVAL 12000 // useconds
  #define FULL_QUEUE_WAIT_TIME 48000 // useconds
#elif RF_RATE == 2
  #define POLLING_INTERVAL 1000 // useconds
  #define FULL_QUEUE_WAIT_TIME 24000 // useconds
#else
  #error "RF_RATE must be defined and be 0, 1, or 2"
#endif

namespace xbee_driver {

XbeeDriver::XbeeDriver() : Node("xbee_driver") {
    RCLCPP_INFO(get_logger(), "Starting XBee driver...");

    conn_ = new XBeeConnection(XbeeBajaSerialConfig::CAR_DEVICE, XbeeBajaSerialConfig::BAJA_BAUDRATE, XbeeBajaSerialConfig::CONGESTION_CONTROL_WINDOW);
    int err = conn_->open();
    while (err == Connection::RECOVERABLE_ERROR) {
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
        err = conn_->open();
    }
    if (err == Connection::IRRECOVERABLE_ERROR) {
        RCLCPP_ERROR(get_logger(), "XBee connection could not be opened, exiting");
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(get_logger(), "XBee connection initialized");

    tx_sub_ = create_subscription<std_msgs::msg::String>(
        "radio_data", 10, std::bind(&XbeeDriver::try_transmit_data, this, std::placeholders::_1));

    rx_pub_ = create_publisher<std_msgs::msg::String>("xbee_rx_data", 10);

    timer_ = create_wall_timer(std::chrono::microseconds(POLLING_INTERVAL), std::bind(&XbeeDriver::timer_callback, this));
}

XbeeDriver::~XbeeDriver() {
    delete conn_;
}

void XbeeDriver::timer_callback() {
    RCLCPP_INFO(get_logger(), "Spinning");
    try_receive_data();
}

int XbeeDriver::try_transmit_data(const std_msgs::msg::String& msg) {
    if ( msg.data.empty() ){
        RCLCPP_INFO(get_logger(), "No message to transmit");
        return EXIT_SUCCESS;
    }
    RCLCPP_INFO(get_logger(), "Retrieved message from subscription, sending to XBee");

    int err = conn_->send(msg.data);
    int iter = 1;
    while (iter <= XBEE_DRIVER_MAX_SEND_RETRIES && err == Connection::QUEUE_FULL) {
        RCLCPP_WARN(get_logger(), "XBee serial queue full, waiting and retrying");
        std::this_thread::sleep_for(std::chrono::microseconds(FULL_QUEUE_WAIT_TIME));
        err = conn_->send(msg.data);
        iter++;
    }

    if (err == Connection::QUEUE_FULL) {
        RCLCPP_WARN(get_logger(), "XBee queue full and exceeded transmit retries, could not send");
        return EXIT_SUCCESS;
    }

    if (err == Connection::SEND_FAILED) {
        RCLCPP_WARN(get_logger(), "Send failed for unknown reason, could not send");
    }

    if (err == Connection::MSG_TOO_LARGE) {
        RCLCPP_WARN(get_logger(), "Message too large, could not send");
        return EXIT_SUCCESS;
    }

    if (err == Connection::IRRECOVERABLE_ERROR) {
        RCLCPP_ERROR(get_logger(), "Could not send because connection failed");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int XbeeDriver::try_receive_data() {
    if (conn_->num_messages_available() <= 0) {
        int err = conn_->tick();
        if (err == Connection::IRRECOVERABLE_ERROR) {
            RCLCPP_ERROR(get_logger(), "Failed to tick device");
            return EXIT_FAILURE;
        }
    }

    if (conn_->num_messages_available() > 0) {
        RCLCPP_INFO(get_logger(), "Message available from XBee");
        std::string msg_str = conn_->pop_message();
        std_msgs::msg::String msg;
        msg.data = msg_str;
        rx_pub_->publish(msg);
    }
    return EXIT_SUCCESS;
}

} // namespace xbee_driver
