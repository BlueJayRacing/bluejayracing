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
  xbee_frame_handlers = new xbee_dispatch_table_entry_t[] {
    {XBEE_FRAME_TRANSMIT_STATUS, 0, &XBeeConnection::tx_status_handler, this},
    {XBEE_FRAME_RECEIVE, 0, &XBeeConnection::receive_handler, this},
    XBEE_FRAME_HANDLE_LOCAL_AT,
    XBEE_FRAME_TABLE_END
  };
}

XBeeConnection::~XBeeConnection()
{
  delete rx_queue;
  delete xbee_frame_handlers;
}

Connection::Status XBeeConnection::open()
{
  xbee_dev_t xbee;
  int err = init_baja_xbee();
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

Connection::Status XBeeConnection::tx_status()
{
  this->tick();
  return this->send_succeeded ? SUCCESS : RECOVERABLE_ERROR;
}

Connection::Status XBeeConnection::send(const std::string msg)
{
  if (msg.length() > XbeeBajaNetworkConfig::MAX_PAYLOAD_SIZE)
  {
    return MSG_TOO_LARGE;
  }
  
  // XBee library expects char* for payload
  int payload_size = msg.length();
  char payload[payload_size + 1];
  std::strcpy(payload, msg.c_str());

  this->latest_frame_id += 1;
  xbee_header_transmit_explicit_t frame_out_header = {
    .frame_type = XBEE_FRAME_TRANSMIT_EXPLICIT,
    .frame_id = this->latest_frame_id,
    .ieee_address = *WPAN_IEEE_ADDR_BROADCAST,
    .network_address_be = 0xFFFE, // "Reserved"
    .source_endpoint = WPAN_ENDPOINT_DIGI_DATA,
    .dest_endpoint = WPAN_ENDPOINT_DIGI_DATA,
    .cluster_id_be = DIGI_CLUST_SERIAL,
    .profile_id_be = WPAN_PROFILE_DIGI,
    .broadcast_radius = 0x0,
    .options = 0x0,
  };

  int err = xbee_frame_write(&xbee, &frame_out_header, sizeof(frame_out_header), payload, payload_size, 0);
  if (err == -EBUSY)
  {
    std::cout << "TX Serial Queue full" << std::endl;
    return QUEUE_FULL;
  }
  if (err == -EMSGSIZE) {
    std::cout << "Cannot transmit, message too large" << std::endl;
    return MSG_TOO_LARGE; // Can never send a msg this large
  }
  if (err == EINVAL) {
    std::cout << "Irrecoverable error when transmitting" << std::endl;
    return IRRECOVERABLE_ERROR;
  }
  return SUCCESS;
}

Connection::Status XBeeConnection::tick()
{
  int err = xbee_dev_tick(&xbee);
  if (err >= 1)
  {
    return SUCCESS;
  }
  if (err < 0)
  {
    std::cout << "ERROR: Could not tick device: " << -err << std::endl;
    return IRRECOVERABLE_ERROR;
  }
  return SUCCESS; // No frames to dispatch
}

int XBeeConnection::num_messages_available() const
{
  return this->rx_queue->size();
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
  XBeeConnection *this_conn = (XBeeConnection *) conn_context;
  const xbee_frame_transmit_status_t *frame = (const xbee_frame_transmit_status_t*) raw;
  
  if (frame == nullptr)
  {
    std::cout << "Received null TX status frame" << std::endl;
    return -EINVAL;
  }

  if (frame_length < offsetof(xbee_frame_transmit_status_t, delivery))
  {
    std::cout << "Received TX status frame too short" << std::endl;
    return -EBADMSG;
  }

  if (frame->delivery == XBEE_TX_DELIVERY_SUCCESS)
  {
    this_conn->send_succeeded = false;
    return -EBADMSG;
  } else 
  {
    this_conn->send_succeeded = true;
  }
  return 0;
}

int XBeeConnection::receive_handler(xbee_dev_t *xbee, const void FAR *raw,
                                    uint16_t frame_length, void FAR *conn_context)
{
  XBeeConnection *this_conn = (XBeeConnection *)conn_context;
  const xbee_frame_receive_t *frame = (const xbee_frame_receive_t *)raw;

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

  std::string payload((char *)frame->payload, payload_len);
  this_conn->rx_queue->push(payload);
  return EXIT_SUCCESS;
}

Connection::Status XBeeConnection::init_baja_xbee()
{
  this->serial = XBeeConnection::init_serial();

  // Dump state to stdout for debug
  int err = xbee_dev_init(&xbee, &serial, NULL, NULL, xbee_frame_handlers);
  if (err)
  {
    std::cout << "Error initializing XBee device" << std::endl;
    return IRRECOVERABLE_ERROR;
  }
  std::cout << "Initialized XBee device abstraction" << std::endl;

  // Need to initialize AT layer so we can transmit
  err = xbee_cmd_init_device(&xbee);
  if (err)
  {
    std::cout << "Error initializing AT layer" << std::endl;
    return IRRECOVERABLE_ERROR;
  }
  do
  {
    xbee_dev_tick(&xbee);
    err = xbee_cmd_query_status(&xbee);
  } while (err == -EBUSY);
  if (err)
  {

    printf("Error %d waiting for AT init to complete.\n", err);
  }

  std::cout << "Initialized XBee AT layer" << std::endl;
  xbee_dev_dump_settings(&xbee, XBEE_DEV_DUMP_FLAG_DEFAULT);
  std::cout << "Verifying XBee settings conform to Baja standard..." << std::endl;

  err = XBeeConnection::write_baja_settings(&xbee);
  if (err == IRRECOVERABLE_ERROR)
  {
    std::cout << "Xbee settings were not verified" << std::endl;
    return IRRECOVERABLE_ERROR;
  }
  xbee_dev_tick(&xbee);
  std::cout << "XBee settings conform to Baja standards" << std::endl
            << std::endl;
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
    sizeof(serial.device)
  );
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