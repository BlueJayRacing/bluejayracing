#include <baja_msgs/msg/observation.hpp>
#include <rclcpp/rclcpp.hpp>

namespace car
{
class UartEmit : public rclcpp::Node {
  public:
    UartEmit();

  private:
    void observation_callback(const baja_msgs::msg::Observation msg);
    void emit_rtk_correction(const baja_msgs::msg::Observation msg);
    rclcpp::Subscription<baja_msgs::msg::Observation>::SharedPtr subscription_;
    int serial_port;
};
} // namespace car
