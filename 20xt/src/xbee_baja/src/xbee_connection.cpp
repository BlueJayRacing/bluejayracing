#include "xbee_connection.h"
#include "xbee_baja_config.h"

extern "C" {
  #include "xbee/device.h"
  #include "xbee/atcmd.h"
  #include "xbee/wpan.h"
  #include "platform_config.h"
}

// TODO: write the frame handlers


XBeeConnection::XBeeConnection(){
  // TODO: dynamically allocate the RX queue
}

XBeeConnection::~XBeeConnection(){
  // TODO: delete the RX queue
}


Connection::Status XBeeConnection::open(){
  return Connection::IRRECOVERABLE_ERROR; // TODO: remove after implementation

  const xbee_dispatch_table_entry_t xbee_frame_handlers[] = {
    {XBEE_FRAME_TRANSMIT_STATUS, 0, this->tx_status_handler, nullptr},
    {XBEE_FRAME_RECEIVE, 0, this->receive_handler, nullptr},
    XBEE_FRAME_HANDLE_LOCAL_AT,
    XBEE_FRAME_TABLE_END};

  // TODO: Initialize this->xbee using the init_baja_xbee() function
  //       ... (see testConfigureXbee.h/c for example)

  this->conn_open = true;
  return Connection::SUCCESS;
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

Connection::Status XBeeConnection::tick() {
  return Connection::IRRECOVERABLE_ERROR;
  // TODO: Tick the xbee, handle possible failures
}

int XBeeConnection::num_messages_available() const{
  return rx_queue->size();
}

std::string XBeeConnection::pop_message() {
  if (rx_queue->size() == 0) {
    return NULL;
  }
  
  std::string msg = rx_queue->front();
  rx_queue->pop();
  return msg;
}


