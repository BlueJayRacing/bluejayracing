#include <string>
#include <iostream>
#include <mqueue.h>
#include <unistd.h>

#include "mains/xbee_driver.h"
#include "helpers/ipc_config.h"
#include "interfaces/connection.h"
#include "xbee/xbee_connection.h"
#include "xbee/xbee_baja_serial_config.h"

#define XBEE_DRIVER_MAX_SEND_RETRIES 2

int main() {
  std::cout << "starting xbee driver..." << std::endl;
  
  // Open the Xbee connection
  Connection* conn = new XBeeConnection(XbeeBajaSerialConfig::STATION_DEVICE, XbeeBajaSerialConfig::BAUDRATE);
  int err = conn->open();
  while (err == Connection::RECOVERABLE_ERROR) {
    usleep(100000);
    err = conn->open(); // can try again
  }
  if (err == Connection::IRRECOVERABLE_ERROR) {
    std::cout << "Xbee connection could not be opened, exiting" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Xbee connection initialized" << std::endl;

  // Open the IPC queues
  const mqd_t tx_queue = StationIPC::open_queue(StationIPC::XBEE_DRIVER_TO_TX_QUEUE, false);
  const mqd_t rx_queue = StationIPC::open_queue(StationIPC::XBEE_DRIVER_RX_QUEUE, false);

  while (true) {
    usleep(100000);
    // Send
    err = try_transmit_data(conn, tx_queue);
    if (err == EXIT_FAILURE) {
      std::cout << "Connection failed when trying to transmit, exiting" << std::endl;
      return EXIT_FAILURE;
    }

    // Recieve
    err = try_recieve_data(conn, rx_queue);
    if (err == EXIT_FAILURE) {
      std::cout << "Connection failed when trying to recieve, exiting" << std::endl;
      return EXIT_FAILURE;
    }
  }
 
  // Cleanup
  delete conn;
  return EXIT_SUCCESS;
}

/* Makes best effort to send a message if messages are available (retries on failure) */
int try_transmit_data(Connection* conn, const mqd_t tx_queue) {

  std::string msg = StationIPC::get_message(tx_queue);
  if (msg.empty()) {
    return EXIT_SUCCESS;
  }
  std::cout << "Retrieved message from IPC, sending to Xbee" << std::endl;

  // If full recoverable, wait only once
  int err = conn->send(msg);
  for (int iter = 2; iter <= XBEE_DRIVER_MAX_SEND_RETRIES || err == Connection::QUEUE_FULL || err == Connection::SEND_FAILED; iter++) {
   usleep(100000);
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

/* Attmepts to recieve data, dispatches if data available */
int try_recieve_data(Connection* conn, const mqd_t rx_queue) {
  if (conn->num_messages_available() <= 0) {
    int err = conn->tick();
    if (err == Connection::IRRECOVERABLE_ERROR) {
      std::cout << "Failed to tick device" << std::endl;
      return EXIT_FAILURE;
    }
  }


  if (conn->num_messages_available() > 0) {
    std::cout << "Message available from Xbee" << std::endl;
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