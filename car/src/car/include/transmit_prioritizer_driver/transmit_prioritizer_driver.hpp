#pragma once

#include <baja_msgs/msg/observation.hpp>
#include <proto/baja_live_comm.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace transmit_prioritizer
{

class TransmitPrioritizerDriver : public rclcpp::Node {
  public:
    TransmitPrioritizerDriver();

  private:
    void observation_callback(const baja_msgs::msg::Observation::SharedPtr msg);
    void translate_observation(const baja_msgs::msg::Observation& ros_observation, Observation& proto_observation);
    void translate_timestamp(const baja_msgs::msg::Timestamp& ros_timestamp, Timestamp& proto_timestamp);
    void translate_gps(const baja_msgs::msg::GPS& ros_gps, GPS& proto_gps);
    void translate_localization(const baja_msgs::msg::Localization& ros_localization, Localization& proto_localization);
    void translate_communication(const baja_msgs::msg::Communication& ros_communication,
                                 Communication& proto_communication);
    void translate_analog_channel(const baja_msgs::msg::AnalogChannel& ros_analog_channel,
                                  AnalogChannel& proto_analog_channel);
    void translate_car_state(const baja_msgs::msg::CarState& ros_car_state, CarState& proto_car_state);
    void translate_rtk_correction(const baja_msgs::msg::RTKCorrection& ros_rtk_correction,
                                  RTKCorrection& proto_rtk_correction);

    rclcpp::Subscription<baja_msgs::msg::Observation>::SharedPtr observation_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr radio_pub_;

    LiveComm live_comm_;
    const size_t MAX_PAYLOAD_SIZE = 200;

    std::string serializeDoubleToBinaryString(double value);
};

} // namespace transmit_prioritizer