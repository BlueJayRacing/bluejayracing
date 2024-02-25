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

#define RF_RATE 0 // 0: 10 kb/s, 1: 110 kb/s, 2: 250 kb/s

#ifndef RF_RATE
    #error "RF_RATE must be defined and be 0, 1, or 2"
#endif
#if RF_RATE == 0
  #define POLLING_INTERVAL 120000 // useconds
  #define FULL_QUEUE_WAIT_TIME 1200000 // useconds
#elif RF_RATE == 1
  #define POLLING_INTERVAL 20000 // useconds
  #define FULL_QUEUE_WAIT_TIME 200000 // useconds
#elif RF_RATE == 2
  #define POLLING_INTERVAL 10000 // useconds
  #define FULL_QUEUE_WAIT_TIME 100000 // useconds
#else
  #error "RF_RATE must be defined and be 0, 1, or 2"
#endif

int main() {
  std::cout << "starting xbee driver..." << std::endl;
  
  // Open the Xbee connection
  Connection* conn = new XBeeConnection(XbeeBajaSerialConfig::STATION_DEVICE, XbeeBajaSerialConfig::BAUDRATE, XbeeBajaSerialConfig::CONGESTION_CONTROL_WINDOW);
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
    usleep(POLLING_INTERVAL);
    // Send
    err = try_transmit_data(conn, tx_queue);
    if (err == EXIT_FAILURE) {
      std::cout << "Connection failed when trying to transmit, exiting" << std::endl;
      return EXIT_FAILURE;
    }

    // Recieve
    // err = try_recieve_data(conn, rx_queue);
    // if (err == EXIT_FAILURE) {
    //   std::cout << "Connection failed when trying to recieve, exiting" << std::endl;
    //   return EXIT_FAILURE;
    // }
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
  int iter = 1;
  while (iter <= XBEE_DRIVER_MAX_SEND_RETRIES && err == Connection::QUEUE_FULL) {
    std::cout << "Xbee serial queue full, waiting and retrying" << std::endl;
    usleep(FULL_QUEUE_WAIT_TIME);
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