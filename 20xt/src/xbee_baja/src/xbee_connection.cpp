#include <cstring> // for memset
#include <cstddef>
#include <iostream>
#include <string>
#include "xbee/xbee_connection.h"
#include "xbee/xbee_baja_network_config.h"
#include "xbee/station_serial_config.h"

extern "C"
{
#include "xbee/device.h"
#include "xbee/atcmd.h"
#include "xbee/wpan.h"
#include "platform_config.h"
}

XBeeConnection::XBeeConnection()
{
  rx_queue = new std::queue<std::string>();
}

XBeeConnection::~XBeeConnection()
{
  delete rx_queue;
}

Connection::Status XBeeConnection::open()
{
  const xbee_dispatch_table_entry_t xbee_frame_handlers[] = {
      {XBEE_FRAME_TRANSMIT_STATUS, 0, &XBeeConnection::tx_status_handler, this},
      {XBEE_FRAME_RECEIVE, 0, &XBeeConnection::receive_handler, this},
      XBEE_FRAME_HANDLE_LOCAL_AT,
      XBEE_FRAME_TABLE_END};

  xbee_dev_t xbee;
  int err = init_baja_xbee(&xbee, xbee_frame_handlers);
  if (err != SUCCESS)
  {
    return IRRECOVERABLE_ERROR;
  }

  this->conn_open = true;
  return Connection::SUCCESS;
}

bool XBeeConnection::is_open() const
{
  return conn_open;
}

void XBeeConnection::close()
{
  // TODO: figure out how to destroy the xbee

  this->conn_open = false;
  return;
}

Connection::Status XBeeConnection::tx_status() const
{
  return IRRECOVERABLE_ERROR;
  // TODO: Check if the XBee's serial buffer is full (return if true)
  // TODO: Check if last message was sent successfully (return if true)
}

Connection::Status XBeeConnection::send(const std::string msg)
{
  return IRRECOVERABLE_ERROR;
  // TODO: convert message to a c-string
  // TODO: Assert that the message is less than MAX_PAYLOAD_SIZE
  // TODO: Check if the XBee's serial buffer is full

  // TODO: Construct the xbee_header_transmit_explicit_t
  // TODO: Send the message using xbee_frame_write()
  // TODO: return success if message sent over serial
}

Connection::Status XBeeConnection::tick()
{
  return IRRECOVERABLE_ERROR;
  // TODO: Tick the xbee, handle possible failures
}

int XBeeConnection::num_messages_available() const
{
  return rx_queue->size();
}

std::string XBeeConnection::pop_message()
{
  if (rx_queue->size() == 0)
  {
    return NULL;
  }

  std::string msg = rx_queue->front();
  rx_queue->pop();
  return msg;
}

int XBeeConnection::tx_status_handler(xbee_dev_t *xbee, const void FAR *raw,
                                      uint16_t frame_length, void FAR *conn_context)
{
  XBeeConnection *this_conn = (XBeeConnection *)conn_context;
  // TODO: handle the transmit status frame
  return 0;
}

int XBeeConnection::receive_handler(xbee_dev_t *xbee, const void FAR *raw,
                                    uint16_t frame_length, void FAR *conn_context)
{
  XBeeConnection *this_conn = (XBeeConnection *) conn_context;
  const xbee_frame_receive_t *frame = (const xbee_frame_receive_t *) raw;

  if (frame == NULL)
  {
    std::cout << "Received null frame" << std::endl;
    return -EINVAL;
  }

  if (frame_length < offsetof(xbee_frame_receive_t, payload))
  {
    std::cout << "Received frame too short" << std::endl;
    return -EBADMSG;
  }

  if (!(frame->options & XBEE_RX_OPT_BROADCAST))
  {
    std::cout << "Received non-broadcast frame" << std::endl;
    return -EBADMSG;
  }

  int payload_len = frame_length - offsetof(xbee_frame_receive_t, payload);

  std::string payload((char*) frame->payload, payload_len);
  this_conn->rx_queue->push(payload);
  return EXIT_SUCCESS;
}

Connection::Status XBeeConnection::init_baja_xbee(xbee_dev_t *xbee, const xbee_dispatch_table_entry_t *xbee_frame_handlers)
{
  xbee_serial_t serial = XBeeConnection::init_serial();

  // Dump state to stdout for debug
  int err = xbee_dev_init(xbee, &serial, NULL, NULL, xbee_frame_handlers);
  if (err)
  {
    std::cout << "Error initializing XBee device" << std::endl;
    return IRRECOVERABLE_ERROR;
  }
  std::cout << "Initialized XBee device abstraction" << std::endl;

  // Need to initialize AT layer so we can transmit
  err = xbee_cmd_init_device(xbee);
  if (err)
  {
    std::cout << "Error initializing AT layer" << std::endl;
    return IRRECOVERABLE_ERROR;
  }
  do
  {
    xbee_dev_tick(xbee);
    err = xbee_cmd_query_status(xbee);
  } while (err == -EBUSY);
  if (err)
  {
    
    printf("Error %d waiting for AT init to complete.\n", err);
  }

  std::cout << "Initialized XBee AT layer" << std::endl;
  xbee_dev_dump_settings(xbee, XBEE_DEV_DUMP_FLAG_DEFAULT);
  std::cout << "Verifying XBee settings conform to Baja standard..." << std::endl;

  err = XBeeConnection::write_baja_settings(xbee);
  if (err == IRRECOVERABLE_ERROR)
  {
    std::cout << "Xbee settings were not verified" << std::endl;
    return IRRECOVERABLE_ERROR;
  }
  xbee_dev_tick(xbee);
  std::cout << "XBee settings conform to Baja standards" << std::endl << std::endl;
  return SUCCESS;
}

xbee_serial_t XBeeConnection::init_serial()
{
  // We want to start with a clean slate
  xbee_serial_t serial;
  std::memset(&serial, 0, sizeof(serial));

  // Set the baudrate and device ID.
  serial.baudrate = StationSerialXbeeConfig::XBEE_BAJA_BAUDRATE;
  std::strncpy(
      serial.device,
      StationSerialXbeeConfig::LINUX_SERIAL_DEVICE_ID.c_str(),
      sizeof(serial.device));
  return serial;
}

Connection::Status XBeeConnection::write_baja_settings(xbee_dev_t *xbee)
{
  // TODO: THis function is broken
  // int err;
  // // TODO: set channel mask (CM), which doesn't have a 32-bit value?

  // for (int i = 0; i < sizeof XBEE_BAJA_CONFIGS / sizeof XBEE_BAJA_CONFIGS[0]; i++) {
  //   do {
  //     err = xbee_cmd_simple(xbee, XBEE_BAJA_CONFIGS[i].name, XBEE_BAJA_CONFIGS[i].value);
  //   } while (err == -EBUSY);
  //   if (err == -EINVAL) {
  //     printf("Error sending %s command. Invalid parameter\n", XBEE_BAJA_CONFIGS[i].name);
  //     return EXIT_FAILURE;
  //   }
  //   printf("Set %s to %d\n", XBEE_BAJA_CONFIGS[i].name, XBEE_BAJA_CONFIGS[i].value);
  // }
  // xbee_cmd_execute(xbee, "WR", NULL, 0);
  return SUCCESS;
}