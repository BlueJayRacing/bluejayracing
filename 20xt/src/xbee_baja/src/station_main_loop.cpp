#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include "connection.h"
#include "xbee_connection.h"
#include "baja_live_comm.pb.h"
#include "trx_queues.h"
#include "queue_manage.h"
#include "station_driver.h"


int station_main_loop(void) {

  // Construct RX queues using multiqueue (create one for each field ID)
  // Construct TX queues using multiqueue (create one for each field ID)
  TRXProtoQueues* tx_queues = new TRXProtoQueues();
  TRXProtoQueues* rx_queues = new TRXProtoQueues();

  // Construct a connection, open it
  Connection* conn = new XBeeConnection();
  int err = conn->open();
  while (err == Connection::RECOVERABLE_ERROR) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    err = conn->open(); // can try again
  }
  if (err == Connection::IRRECOVERABLE_ERROR) {
    std::cout << "Connection failed, exiting" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Connection initialized" << std::endl;

  while (true) {
    // Send
    err = try_produce_data(conn, tx_queues);
    if (err == EXIT_FAILURE) {
      std::cout << "Connection failed, exiting" << std::endl;
      return EXIT_FAILURE;
    }

    // Recieve
    err = try_consume_data(conn, rx_queues);
    if (err = EXIT_FAILURE) {
      std::cout << "Connection failed, exiting" << std::endl;
      return EXIT_FAILURE;
    }
  }
 
  // Cleanup
  delete conn, tx_queues, rx_queues;
}

int try_produce_data(Connection* conn, TRXProtoQueues* tx_queues) {
  if (num_payloads_available(tx_queues) == 0) {
    return EXIT_SUCCESS;
  }

  LiveComm msg = build_message(tx_queues);
  int err = conn->send(msg.SerializeAsString());

  // If full recoverable, wait only once
  for (int iter = 2; iter <= MAX_SEND_RETRIES || err == Connection::QUEUE_FULL || err == Connection::SEND_FAILED; iter++) {
    std::cout << "Send failed, retrying" << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(1));
    err = conn->send(msg.SerializeAsString());
    iter++;
  }

  if (err == Connection::QUEUE_FULL || err == Connection::SEND_FAILED) {
    std::cout << "Send failed, could not send" << std::endl;
    return EXIT_SUCCESS; // non-fatal error
  }

  if (err == Connection::MSG_TOO_LARGE) {
    std::cout << "Message too large, could not send" << std::endl;
    return EXIT_SUCCESS;
  }
  
  // Any connection failure is fatal
  if (err == Connection::CONNECTION_BROKEN) {
    std::cout << "Connection failed, exiting" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS
}


int try_recieve_data(Connection* conn, TRXProtoQueues* tx_queues) {
  if (conn->num_messages_available() <= 0) {
    err = conn->tick();
    if (err == Connection::CONNECTION_BROKEN) {
      std::cout << "Connection failed, exiting" << std::endl;
      return EXIT_FAILURE;
    }
  }

  if (conn->num_messages_available() > 0) {
    std::string encoded_msg = conn->pop_message();
    LiveComm decoded_msg;
    decoded_msg.ParseFromString(msg);
    distribute_message(decoded_msg, rx_queues);
  }
}