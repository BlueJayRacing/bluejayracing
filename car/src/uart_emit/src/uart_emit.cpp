// how to configure serial IO: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
#include <uart_emit/uart_emit.hpp>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_PORT "/dev/ttyACM0"

using namespace std;
using namespace car;

UartEmit::UartEmit() : Node("uart_emit_node")
{
    cout << "Starting uart emit for rtcm corrections..." << endl;

    // Open UART Port
    serial_port = open(SERIAL_PORT, O_RDWR);

    // Check for errors
    if (serial_port < 0)
    {
        RCLCPP_ERROR(get_logger(), "Error %i from open: %s", errno, strerror(errno));
        return;
    }

    // Create subscriber
    subscription_ = create_subscription<baja_msgs::msg::Observation>(
        "observation_topic", 10, std::bind(&UartEmit::observation_callback, this, std::placeholders::_1));
}

void UartEmit::observation_callback(const baja_msgs::msg::Observation msg)
{
    emit_rtk_correction(msg);
}

void UartEmit::emit_rtk_correction(const baja_msgs::msg::Observation msg)
{

    auto rtk = msg.rtk_correction[0];
    int ts = msg.timestamp.ts;

    const uint8_t* rtk_msg = reinterpret_cast<const uint8_t*>(rtk.rtk_correction.data());
    size_t rtk_msg_size = rtk.rtk_correction.size();


    write(serial_port, rtk_msg, rtk_msg_size);

    for (const auto& e : rtk.rtk_correction)
        cout << hex << static_cast<int>(e) << " ";
    cout << endl;
}