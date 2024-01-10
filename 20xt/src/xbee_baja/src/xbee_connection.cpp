#include <cstring> // for memset
#include "xbee/xbee_connection.h"
#include "xbee/xbee_baja_network_config.h"
#include "xbee/station_serial_config.h"

extern "C" {
  #include "xbee/device.h"
  #include "xbee/atcmd.h"
  #include "xbee/wpan.h"
  #include "platform_config.h"
}

XBeeConnection::XBeeConnection(){
  rx_queue = new std::queue<std::string>();
}

XBeeConnection::~XBeeConnection(){
  delete rx_queue;
}

Connection::Status XBeeConnection::open(){
  const xbee_dispatch_table_entry_t xbee_frame_handlers[] = {
    {XBEE_FRAME_TRANSMIT_STATUS, 0, &XBeeConnection::tx_status_handler, this},
    {XBEE_FRAME_RECEIVE, 0, &XBeeConnection::receive_handler, this},
    XBEE_FRAME_HANDLE_LOCAL_AT,
    XBEE_FRAME_TABLE_END};

  xbee_dev_t xbee;
  int err = init_baja_xbee(&xbee, xbee_frame_handlers);
  if (err != SUCCESS) {
    return IRRECOVERABLE_ERROR;
  }
  
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
  return IRRECOVERABLE_ERROR;
  // TODO: Check if the XBee's serial buffer is full (return if true)
  // TODO: Check if last message was sent successfully (return if true)
}

Connection::Status XBeeConnection::send(const std::string msg){
  return IRRECOVERABLE_ERROR;
  // TODO: convert message to a c-string
  // TODO: Assert that the message is less than MAX_PAYLOAD_SIZE
  // TODO: Check if the XBee's serial buffer is full

  // TODO: Construct the xbee_header_transmit_explicit_t
  // TODO: Send the message using xbee_frame_write()
  // TODO: return success if message sent over serial
}

Connection::Status XBeeConnection::tick() {
  return IRRECOVERABLE_ERROR;
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

static int XBeeConnection::tx_status_handler(xbee_dev_t *xbee, const void FAR *raw, 
                    uint16_t length, void FAR *conn_context) {
  XBeeConnection * this_conn = (XBeeConnection *) conn_context;
  // TODO: handle the transmit status frame
  return 0;
}

static int XBeeConnection::receive_handler(xbee_dev_t *xbee, const void FAR *raw, 
                      uint16_t length, void FAR *conn_context) {
  XBeeConnection* this_conn = (XBeeConnection *) conn_context;
  // TODO: handle the receive frame, enqueueing it in this_conn->rx_queue
  return 0;
}

static int XBeeConnection::init_baja_xbee(xbee_dev_t *xbee, const xbee_dispatch_table_entry_t* xbee_frame_handlers)
{
  xbee_serial_t serial = init_serial();

  // Dump state to stdout for debug
  int err = xbee_dev_init(xbee, &serial, NULL, NULL, xbee_frame_handlers);
  if (err)
  {
    printf("Error initializing abstraction: %" PRIsFAR "\n", strerror(-err));
    return EXIT_FAILURE;
  }
  printf("Initialized XBee device abstraction.\n");

  // Need to initialize AT layer so we can transmit
  err = xbee_cmd_init_device(xbee);
  if (err)
  {
    printf("Error initializing AT layer: %" PRIsFAR "\n", strerror(-err));
    return EXIT_FAILURE;
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

  printf("Initialized XBee AT layer\n");
  xbee_dev_dump_settings(xbee, XBEE_DEV_DUMP_FLAG_DEFAULT);
  printf("Veryfing XBee settings conform to Baja standard...\n");

  err = write_baja_settings(xbee);
  if (err)
  {
    printf("Xbee settings were not verified\n");
    return EXIT_FAILURE;
  }
  xbee_dev_tick(xbee);
  printf("XBee settings conform to Baja standards\n\n");
  return EXIT_SUCCESS;
}

static xbee_serial_t XBeeConnection::init_serial()
{
  // We want to start with a clean slate
  xbee_serial_t serial;
  std::memset(&serial, 0, sizeof(serial));

  // Set the baudrate and device ID.
  serial.baudrate = XBEE_BAJA_BD;
  std::strncpy(serial.device, SERIAL_DEVICE_ID, sizeof(serial.device));
  return serial;
}

static int XBeeConnection::write_baja_settings(xbee_dev_t *xbee)
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
  return EXIT_SUCCESS;
}