// writer_driver.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <baja_msgs/msg/observation.hpp>
// #include <inertial_sense_ros_humble_msgs/msg/didins4.hpp>

namespace writer_driver {

class WriterDriver : public rclcpp::Node {
public:
    WriterDriver();

private:
    void observation_callback(const baja_msgs::msg::Observation::SharedPtr msg);
    void log_gps(const baja_msgs::msg::Observation& observation);
    void log_localization(const baja_msgs::msg::Observation& observation);
    void log_communication(const baja_msgs::msg::Observation& observation);
    void log_analog_ch(const baja_msgs::msg::Observation& observation);
    void log_car_state(const baja_msgs::msg::Observation& observation);
    void log_rtk_correction(const baja_msgs::msg::Observation& observation);
    void write_line(std::ofstream& f, const std::vector<std::string>& v);
    double deserializeDoubleFromBinaryStringCasting(const std::string& buffer);

    rclcpp::Subscription<baja_msgs::msg::Observation>::SharedPtr subscription_;
    // rclcpp::Subscription<inertial_sense_ros_humble_msgs::msg::DIDINS4>::SharedPtr subscription_did_;
    std::vector<std::ofstream> f_;
};

} // namespace writer_driver