

#include "xbee_connection.h"

XBeeConnection::XBeeConnection(){
  
}

XBeeConnection::~XBeeConnection(){
}


Connection::Status XBeeConnection::open(){
  return Connection::IRRECOVERABLE_ERROR;
}

bool XBeeConnection::is_open() const {
  return conn_open;
}

void XBeeConnection::close(){
  
}

Connection::Status XBeeConnection::tx_status() const{
  return Connection::IRRECOVERABLE_ERROR;
}

Connection::Status XBeeConnection::send(const std::string msg){
  return Connection::IRRECOVERABLE_ERROR;
}

Connection::Status XBeeConnection::tick(){
  return Connection::IRRECOVERABLE_ERROR;
}

int XBeeConnection::num_messages_available() const{
  return 0;
}

std::string XBeeConnection::pop_message(){
  return NULL;
}


