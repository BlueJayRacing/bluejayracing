// writer_node.cpp
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <wsg_calibration_driver/wsg_calibration_driver.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wsg_calibration_driver::WSGCalibrationDriver>();
    std::cout << "starting wsg calibration driver..." << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}