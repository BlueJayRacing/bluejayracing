#include <iostream>
#include <mqueue.h>
#include <vector>
#include <unistd.h>


// #include "mains/adc_driver.h"
#include <rclcpp/rclcpp.hpp>
#include <proto/baja_live_comm.pb.h>

// We want to produce a series of dummy Communcation values to the TX queue
int main () {
  // Open the message queue, return if it fails
  mqd_t rx_queue = BajaIPC::open_queue(CarIPC::MQTT_CLIENT_TO_BROKER_QUEUE, false);
  if (rx_queue == -1) {
    std::cout << "Failed to get recieve queue. Errno " << errno << std::endl>    return EXIT_FAILURE;
  }

  while (true) {
    usleep(1000000);
    // Let's send a message!
    //std::string payload = get_my_data_as_string();
    const std::string payload = BajaIPC::get_message(rx_queue);
    std::cout << "Message received: " << payload << std::endl;
  }
}
