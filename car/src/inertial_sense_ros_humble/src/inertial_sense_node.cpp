#include "inertial_sense_ros_humble/inertial_sense_ros_complete.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InertialSenseROS>(false);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->update();
    }
    rclcpp::shutdown();

    return 0;
}