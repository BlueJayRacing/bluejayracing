#pragma once

#include <rclcpp/rclcpp.hpp>
#include <baja_msgs/msg/observation.hpp>
#include <array>
#include <deque>
#include <chrono>

namespace car {

class AxleTorqueAggregator : public rclcpp::Node {
public:
    AxleTorqueAggregator(const std::string& name);

private:
    void observation_callback(const baja_msgs::msg::Observation::SharedPtr msg);
    void update_and_print_averages(const baja_msgs::msg::AnalogChannel& channel);

    rclcpp::Subscription<baja_msgs::msg::Observation>::SharedPtr subscription_;
    std::array<std::deque<double>, 4> axle_torque_buffers_;
    std::array<size_t, 4> message_counters_;
    std::array<std::chrono::steady_clock::time_point, 4> last_print_times_;
    static constexpr size_t MAX_BUFFER_SIZE = 1000;
    static constexpr std::chrono::seconds PRINT_INTERVAL{2};
};

}  // namespace car