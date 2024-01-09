#include "xbee_connection.h"
#include "xbee_baja_config.h"

extern "C" {
  #include "platform_config.h"
  #include "xbee/device.h"
  #include "xbee/atcmd.h"
  #include "xbee/wpan.h"
}

// TODO: write the frame handlers


XBeeConnection::XBeeConnection(){
  // TODO: dynamically allocate the RX queue
}

XBeeConnection::~XBeeConnection(){
  // TODO: delete the RX queue
}


Connection::Status XBeeConnection::open(){
  return Connection::IRRECOVERABLE_ERROR;

  // TODO: Initialize this->xbee using the init_baja_xbee() function
  //       ... (see testConfigureXbee.h/c for example)

  this->conn_open = true
  return CONNECTION::SUCCESS;
}

bool XBeeConnection::is_open() const {
  return conn_open;
}

void XBeeConnection::close(){
  // TODO: figure out how to destroy the xbee
  
  this->conn_open = false;
  return;
}

Connection::Status XBeeConnection::tx_status() const{
  return Connection::IRRECOVERABLE_ERROR;
  // TODO: Check if the XBee's serial buffer is full (return if true)
  // TODO: Check if last message was sent successfully (return if true)
}

Connection::Status XBeeConnection::send(const std::string msg){
  return Connection::IRRECOVERABLE_ERROR;
  // TODO: convert message to a c-string
  // TODO: Assert that the message is less than MAX_PAYLOAD_SIZE
  // TODO: Check if the XBee's serial buffer is full

  // TODO: Construct the xbee_header_transmit_explicit_t
  // TODO: Send the message using xbee_frame_write()
  // TODO: return success if message sent over serial
}

Connection::Status XBeeConnection::tick(){
  return Connection::IRRECOVERABLE_ERROR;
  // TODO: Tick the xbee, handle possible failures
}

int XBeeConnection::num_messages_available() const{
  return rx_queue->size();
}

std::string XBeeConnection::pop_message(){
  return rx_queue->pop();
}


