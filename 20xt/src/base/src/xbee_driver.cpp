#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include "mains/xbee_driver.h"
#include "helpers/ipc_config.h"
#include "interfaces/connection.h"
#include "xbee/xbee_connection.h"

const static int MAX_SEND_RETRIES = 2;

int main() {
  
  // Open the Xbee connection
  Connection* conn = new XBeeConnection();
  int err = conn->open();
  while (err == Connection::RECOVERABLE_ERROR) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    err = conn->open(); // can try again
  }
  if (err == Connection::IRRECOVERABLE_ERROR) {
    std::cout << "Xbee connection could not be opened, exiting" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Xbee connection initialized" << std::endl;

  // Open the IPC queues
  const mqd_t tx_queue = StationIPC::open_queue(StationIPC::XBEE_DRIVER_TO_TX_QUEUE);
  const mqd_t rx_queue = StationIPC::open_queue(StationIPC::XBEE_DRIVER_RX_QUEUE);

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "." << std::endl;

    // Send
    err = _try_transmit_data(conn, tx_queue);
    if (err == EXIT_FAILURE) {
      std::cout << "Connection failed when trying to transmit, exiting" << std::endl;
      return EXIT_FAILURE;
    }

    // Recieve
    err = _try_recieve_data(conn, rx_queue);
    if (err == EXIT_FAILURE) {
      std::cout << "Connection failed when trying to recieve, exiting" << std::endl;
      return EXIT_FAILURE;
    }
  }
 
  // Cleanup
  delete conn;
  return EXIT_SUCCESS;
}

int _try_transmit_data(Connection* conn, const mqd_t tx_queue) {

  std::string msg = StationIPC::get_message(tx_queue);
  if (msg.empty()) {
    return EXIT_SUCCESS;
  }

  // If full recoverable, wait only once
  int err = conn->send(msg);
  for (int iter = 2; iter <= MAX_SEND_RETRIES || err == Connection::QUEUE_FULL || err == Connection::SEND_FAILED; iter++) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
    err = conn->send(msg);
    iter++;
  }

  if (err == Connection::QUEUE_FULL) {
    std::cout << "XBee queue full and exceeded transmit retries, could not send" << std::endl;
    return EXIT_SUCCESS; // non-fatal error
  }

  if (err == Connection::SEND_FAILED) {
    std::cout << "Send failed for unkown reason, could not send" << std::endl;
  }

  if (err == Connection::MSG_TOO_LARGE) {
    std::cout << "Message too large, could not send" << std::endl;
    return EXIT_SUCCESS;
  }
  
  // Any connection failure is fatal
  if (err == Connection::IRRECOVERABLE_ERROR) {
    std::cout << "Could not send because connection failed" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}


int _try_recieve_data(Connection* conn, const mqd_t rx_queue) {
  if (conn->num_messages_available() <= 0) {
    int err = conn->tick();
    if (err == Connection::IRRECOVERABLE_ERROR) {
      std::cout << "Failed to tick device" << std::endl;
      return EXIT_FAILURE;
    }
  }


  if (conn->num_messages_available() > 0) {
    std::string msg = conn->pop_message();
    int err = StationIPC::send_message(rx_queue, msg);

    // Xbee driver responsible for keeping queue recent
    if (err == StationIPC::QUEUE_FULL) {
      StationIPC::get_message(rx_queue);
      StationIPC::send_message(rx_queue, msg);
    }
  }
  return EXIT_SUCCESS;
}