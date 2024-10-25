#include "calibration_printer/calibration_printer.hpp"
#include <numeric>

namespace car
{

AxleTorqueAggregator::AxleTorqueAggregator(const std::string& name) : Node(name), message_counters_{0, 0, 0, 0}
{
    auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group;

    subscription_ = this->create_subscription<baja_msgs::msg::Observation>(
        "/mqtt_data", rclcpp::QoS(1000).best_effort().durability_volatile(),
        std::bind(&AxleTorqueAggregator::observation_callback, this, std::placeholders::_1), options);

    for (auto& time : last_print_times_) {
        time = std::chrono::steady_clock::now();
    }
}

void AxleTorqueAggregator::observation_callback(const baja_msgs::msg::Observation::SharedPtr msg)
{
    for (const auto& channel : msg->analog_ch) {
        if (channel.channel_type >= baja_msgs::msg::AnalogChannel::AXLE_TORQUE_FRONT_LEFT &&
            channel.channel_type <= baja_msgs::msg::AnalogChannel::AXLE_TORQUE_REAR_RIGHT) {
            update_and_print_averages(channel);
        }
    }
}

void AxleTorqueAggregator::update_and_print_averages(const baja_msgs::msg::AnalogChannel& channel)
{
    size_t index          = channel.channel_type - baja_msgs::msg::AnalogChannel::AXLE_TORQUE_FRONT_LEFT;
    auto& buffer          = axle_torque_buffers_[index];
    auto& counter         = message_counters_[index];
    auto& last_print_time = last_print_times_[index];

    buffer.push_back(channel.encoded_analog_points);
    if (buffer.size() > MAX_BUFFER_SIZE) {
        buffer.pop_front();
    }
    counter++;

    auto now = std::chrono::steady_clock::now();
    bool should_print =
        buffer.size() == MAX_BUFFER_SIZE && (counter % MAX_BUFFER_SIZE == 0 || now - last_print_time >= PRINT_INTERVAL);

    if (should_print) {
        double average           = std::accumulate(buffer.begin(), buffer.end(), 0.0) / buffer.size();
        const char* axle_names[] = {"Front Left", "Front Right", "Rear Left", "Rear Right"};
        RCLCPP_INFO(this->get_logger(), "Average Axle Torque (%s): %.2f, Total Messages: %zu", axle_names[index],
                    average, counter);
        last_print_time = now;
    }
}

} // namespace car