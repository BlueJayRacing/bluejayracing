#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include "mains/station_driver.h"
#include "interfaces/connection.h"
#include "xbee/xbee_connection.h"
#include "ipc/trx_queues.h"
#include "ipc/queue_manage.h"
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

    std::cout<<"hi"<<std::endl;

    // Send
    err = try_produce_data(conn, tx_queues);
    if (err == EXIT_FAILURE) {
      std::cout << "Connection failed when trying to transmit, exiting" << std::endl;
      return EXIT_FAILURE;
    }

    // Recieve
    err = try_consume_data(conn, rx_queues);
    if (err == EXIT_FAILURE) {
      std::cout << "Connection failed when trying to recieve, exiting" << std::endl;
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

  // TODO: Revert this test
  // int err = conn->send(msg.SerializeAsString());
  int err = conn->send("Insert my payload here");

  // If full recoverable, wait only once
  for (int iter = 2; iter <= MAX_SEND_RETRIES || err == Connection::QUEUE_FULL || err == Connection::SEND_FAILED; iter++) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
    err = conn->send(msg.SerializeAsString());
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


int try_consume_data(Connection* conn, TRXProtoQueues* rx_queues) {
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
    LiveComm decoded_msg;
    decoded_msg.ParseFromString(encoded_msg);
    distribute_message(decoded_msg, rx_queues);
  }

  return EXIT_SUCCESS;
}