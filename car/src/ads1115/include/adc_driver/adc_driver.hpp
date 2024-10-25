// adc_driver.hpp
#pragma once

#include <ads1115/adc.hpp>
#include <baja_msgs/msg/analog_channel.hpp>
#include <baja_msgs/msg/observation.hpp>
#include <rclcpp/rclcpp.hpp>

namespace adc_driver
{

class ADCDriver : public rclcpp::Node {
  public:
    ADCDriver();

  private:
    void read_and_publish_data();
    std::string serializeDoubleToBinaryString(double value);

    rclcpp::Publisher<baja_msgs::msg::Observation>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    ADC adcs_[5];
    uint8_t channels_[2][5];
};

} // namespace adc_driver