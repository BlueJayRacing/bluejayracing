// writer_node.cpp
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <writer_driver/writer_driver.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<writer_driver::WriterDriver>();
    std::cout << "starting sd writer..." << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}