#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include "mains/station_driver.h"
#include "interfaces/connection.h"
#include "xbee/xbee_connection.h"
#include "crossthread/trx_queues.h"
#include "crossthread/queue_manage.h"
#include "baja_live_comm.pb.h"


int station_main_loop(TRXProtoQueues* tx_queues, TRXProtoQueues* rx_queues) {
  
  // Continue with a normal loop and good practice
  Connection* conn = new XBeeConnection();
  int err = conn->open();
  while (err == Connection::RECOVERABLE_ERROR) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    err = conn->open(); // can try again
  }
  if (err == Connection::IRRECOVERABLE_ERROR) {
    std::cout << "Connection could not be opened, exiting" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Connection initialized" << std::endl;


  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout<<"."<<std::endl;

    // Send
    err = _try_transmit_data(conn, tx_queues);
    if (err == EXIT_FAILURE) {
      std::cout << "Connection failed when trying to transmit, exiting" << std::endl;
      return EXIT_FAILURE;
    }

    // Recieve
    err = _try_recieve_data(conn, rx_queues);
    if (err == EXIT_FAILURE) {
      std::cout << "Connection failed when trying to recieve, exiting" << std::endl;
      return EXIT_FAILURE;
    }
  }
 
  // Cleanup
  delete conn;
  return EXIT_SUCCESS;
}

int _try_transmit_data(Connection* conn, TRXProtoQueues* tx_queues) {
  if (num_payloads_available(tx_queues) == 0) {
    return EXIT_SUCCESS;
  }

  std::string msg = build_message(tx_queues);
  int err = conn->send(msg);

  // If full recoverable, wait only once
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


int _try_recieve_data(Connection* conn, TRXProtoQueues* rx_queues) {
  if (conn->num_messages_available() <= 0) {
    int err = conn->tick();
    if (err == Connection::IRRECOVERABLE_ERROR) {
      std::cout << "Failed to tick device" << std::endl;
      return EXIT_FAILURE;
    }
  }

  if (conn->num_messages_available() > 0) {
    std::string encoded_msg = conn->pop_message();
    std::cout << encoded_msg << std::endl;
    Observation decoded_msg;
    decoded_msg.ParseFromString(encoded_msg);
    distribute_message(decoded_msg, rx_queues);
  }

  return EXIT_SUCCESS;
}