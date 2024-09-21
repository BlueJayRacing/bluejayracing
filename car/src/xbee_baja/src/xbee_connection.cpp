#include <cstring> // for memset
#include <cstddef>
#include <iostream>
#include <string>
#include "xbee/xbee_connection.h"
#include "xbee/xbee_baja_network_config.h"
#include "xbee/xbee_baja_serial_config.h"

extern "C"
{
#include "xbee/device.h"
#include "xbee/atcmd.h"
#include "xbee/wpan.h"
#include "xbee/apply_profile.h"
#include "platform_config.h"
}

XBeeConnection::XBeeConnection(const std::string serial_device, const int baudrate, const int cwnd_size)
  : serial_device(serial_device), baudrate(baudrate), CWND_SIZE(cwnd_size), num_queued_for_tx(0), last_acked_frame_id(0)
{
  // Frame handlers must be dynamically allocated so that
  // the xbee library can access them
  this->rx_queue = new std::queue<std::string>();
  this->xbee_frame_handlers = new xbee_dispatch_table_entry_t[] {
    {XBEE_FRAME_TRANSMIT_STATUS, 0, &XBeeConnection::tx_status_handler, this},
    {XBEE_FRAME_RECEIVE, 0, &XBeeConnection::receive_handler, this},
    XBEE_FRAME_HANDLE_LOCAL_AT,
    XBEE_FRAME_TABLE_END
  };
}

XBeeConnection::~XBeeConnection()
{
  this->close();
  delete this->rx_queue;
  delete this->xbee_frame_handlers;
}

Connection::Status XBeeConnection::open()
{
  /* if can't connect, assume default settings and attempt to write baja settings */
  if (this->init_baja_xbee() != SUCCESS) {
    std::cout << "could not init baja xbee, attempting to init factory xbee" << std::endl;
    if (this->init_factory_xbee() != SUCCESS)
    {
      std::cerr << "could not init default xbee" << std::endl;
      return IRRECOVERABLE_ERROR;
    }
    if (this->write_baja_settings() != SUCCESS)
    {
      std::cerr << "could not write baja settings" << std::endl;
      return IRRECOVERABLE_ERROR;
    }
  }
  this->conn_open = true;
  return Connection::SUCCESS;
}

bool XBeeConnection::is_open() const
{
  return this->conn_open;
}

void XBeeConnection::close()
{
  xbee_ser_close(&(this->serial));
  this->conn_open = false;
  return;
}

int XBeeConnection::num_msgs_queued_for_tx()
{
  this->tick();
  return this->num_queued_for_tx;
}

Connection::Status XBeeConnection::send(const std::string msg)
{
  if (msg.length() > XBEE_BAJA_MAX_PAYLOAD_SIZE)
  {
    return MSG_TOO_LARGE;
  }

  if (this->num_queued_for_tx >= this->CWND_SIZE)
  {
    std::cout << "window full, so ticking" << std::endl;
    this->tick();
    if (this->num_queued_for_tx >= this->CWND_SIZE)
    {
      return QUEUE_FULL; // Still full after tick :(
    }
  }
  
  // XBee library expects char* for payload
  int payload_size = msg.length();
  char payload[payload_size + 1];
  std::strcpy(payload, msg.c_str());

  // We're actually sending a 0x10 transmit (not explicit), but 
  // the library doesn't directly implement this frame type. isntead
  // we use the explicit frame type and set destination address to
  // broadcast
  uint8_t frame_id = this->last_acked_frame_id + this->num_queued_for_tx + 1;
  xbee_header_transmit_explicit_t frame_out_header = {
    .frame_type = XBEE_FRAME_TRANSMIT_EXPLICIT,
    .frame_id = frame_id,
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

  this->num_queued_for_tx += 1;
  return SUCCESS;
}

Connection::Status XBeeConnection::tick()
{
  int err = xbee_dev_tick(&xbee);
  if (err == -EINVAL)
  {
    std::cout << "Could not tick XBee, not a valid XBee obj" << std::endl;
    return IRRECOVERABLE_ERROR;
  }
  if (err == -EBUSY)
  {
    std::cout << "Could not tick XBee, already being ticked" << std::endl;
    return RECOVERABLE_ERROR;
  }
  if (err == -EIO)
  {
    std::cout << "Could not tick XBee, error with serial port" << std::endl;
    return IRRECOVERABLE_ERROR;
  }
  return SUCCESS; // No frames to dispatch
}

int XBeeConnection::num_messages_available() const
{
  // Caution, we don't tick before checking if messages are available,
  // the returned value may be out of date
  return this->rx_queue->size();
}

std::string XBeeConnection::pop_message()
{
  if (this->rx_queue->size() == 0)
  {
    return NULL;
  }

  std::string msg = this->rx_queue->front();
  this->rx_queue->pop();
  return msg;
}

int XBeeConnection::tx_status_handler(xbee_dev_t *xbee, const void FAR *raw,
                                      uint16_t frame_length, void FAR *conn_context)
{
  XBeeConnection *this_conn = (XBeeConnection *) conn_context;
  const xbee_frame_transmit_status_t *frame = (const xbee_frame_transmit_status_t*) raw;
  
  if (frame == nullptr)
  {
    std::cerr << "Received null TX status frame" << std::endl;
    return -EINVAL;
  }

  if (frame_length < offsetof(xbee_frame_transmit_status_t, delivery))
  {
    std::cerr << "Received TX status frame too short" << std::endl;
    return -EBADMSG;
  }

  if (frame->delivery != XBEE_TX_DELIVERY_SUCCESS)
  {
    std::cerr << "Xbee recieved bad transmit request frame" << std::endl;
    return -EBADMSG;
  }

  // Need to adjust both num outstanding and last acked frame id. Handle the edge case
  // where the frame_id is less than the last acked frame id (i.e. overflow occured)
  unsigned int real_frame_id = frame->frame_id;
  if (frame->frame_id < this_conn->last_acked_frame_id)
  {
    real_frame_id += 256;
  }


  if ((this_conn->last_acked_frame_id + this_conn->num_queued_for_tx) < real_frame_id)
  {
    return 0; // Ack from an old frame, already accounted for
  }
  
  this_conn->num_queued_for_tx -= (real_frame_id - this_conn->last_acked_frame_id);
  this_conn->last_acked_frame_id = frame->frame_id;
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

  // The payload_len is the frame length minus the meta data found
  // at the beginning of the frame
  int payload_len = frame_length - offsetof(xbee_frame_receive_t, payload);
  std::string payload((char *)frame->payload, payload_len);
  this_conn->rx_queue->push(payload);
  return EXIT_SUCCESS;
}

Connection::Status XBeeConnection::init_factory_xbee() {
  this->serial = XBeeConnection::init_serial(this->serial_device, XbeeBajaSerialConfig::FACTORY_BAUDRATE);
  return this->init_after_serial_open();
}

Connection::Status XBeeConnection::init_baja_xbee()
{
  this->serial = XBeeConnection::init_serial(this->serial_device, this->baudrate);
  return this->init_after_serial_open();
}

Connection::Status XBeeConnection::init_after_serial_open() {
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
    return IRRECOVERABLE_ERROR;
  }
  std::cout << "Initialized XBee AT layer" << std::endl;
  xbee_dev_dump_settings(&xbee, XBEE_DEV_DUMP_FLAG_DEFAULT);
  xbee_dev_tick(&xbee);
  return SUCCESS;
}

xbee_serial_t XBeeConnection::init_serial(const std::string serial_device, const int baudrate)
{
  // We want to start with a clean slate
  xbee_serial_t serial;
  std::memset(&serial, 0, sizeof(serial));

  // Set the baudrate and device ID.
  serial.baudrate = baudrate;

  std::strncpy(
    serial.device,
    serial_device.c_str(),
    sizeof(serial.device)
  );
  return serial;
}

/* Write the Baja specific settings and re-open the serial port. Assumes this->xbee is init'd */
Connection::Status XBeeConnection::write_baja_settings()
{
  apply_profile(&this->xbee, XbeeBajaSerialConfig::XBEE_PROFILE_PATH.c_str());

  xbee_ser_close(&(this->serial));
  if (this->init_baja_xbee() != SUCCESS)
  {
    std::cerr << "could not re-init the xbee after writing settings" << std::endl;
    return IRRECOVERABLE_ERROR;
  }
  return SUCCESS;
}