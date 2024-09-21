#include "rtk_corrections.hpp"

namespace baja_rtk {

RTKCorrectionsPublisher::RTKCorrectionsPublisher()
    : Node("rtk_corrections_publisher") {
  publisher_ = create_publisher<baja_msgs::msg::RTKCorrection>("rtk_corrections", 10);
  char port_error = serial_.openDevice(BAJA_RTK_SERIAL_PORT, 115200);
  if (port_error != 1) {
    RCLCPP_ERROR(get_logger(), "Error opening serial port %s", BAJA_RTK_SERIAL_PORT);
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(get_logger(), "Successful connection to %s", BAJA_RTK_SERIAL_PORT);
  }
}

void RTKCorrectionsPublisher::publish_rtk_corrections() {
  std::string rtk_correction = read_raw_rtcm(&serial_);
  auto msg = baja_msgs::msg::RTKCorrection();
  msg.rtk_correction.assign(rtk_correction.begin(), rtk_correction.end());
  publisher_->publish(msg);
}

}  // namespace baja_rtk