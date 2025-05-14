// writer_node.cpp
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <wsg_drive_data_driver/wsg_drive_data_driver.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wsg_drive_data_driver::WSGDriveDataDriver>();
    std::cout << "starting wsg drive data driver..." << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}