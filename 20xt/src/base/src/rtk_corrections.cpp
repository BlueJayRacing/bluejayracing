#include <iostream>
#include <mqueue.h>
#include <vector>
#include <unistd.h>
#include <cassert>

#include "mains/pit_commands.h"
#include "serialib.h"
#include "ipc_config.h"
#include "baja_live_comm.pb.h"



#define SERIAL_PORT "/dev/ttyACM0" // TODO: Change to correct device

std::string read_raw_rtcm(serialib *serial) { // return the first RTCM message it finds
  uint8_t last = 0x00;
  uint8_t cur;

  //int     readBytes   (void *buffer, unsigned int maxNbBytes,const unsigned int timeOut_ms=0, unsigned int sleepDuration_us=100);

  while (true) {
    serial->readBytes((void *) &cur, 1);
    if (last == 0xd3 && (cur >> 2) == 0x00) { // continue reading until a message is found
      break;
    }
    last = cur;
  }

  uint8_t length_buffer;
  serial->readBytes((void *) &length_buffer, 1);

  // calculate length from the 10 bits following d3
  uint16_t length = ((uint16_t) (cur & 0x03) << 8) + length_buffer;

  // read data and parity
  length += 3; // add 3 for parity
  uint8_t *data = (uint8_t *) malloc((length + 3) * sizeof(uint8_t)); // add another 3 for the first three bytes
  try {
    if (serial->readBytes((void *) (data + 3), length) != length) { // TODO: Jonathan, can you check this line? I had to fix it
      throw -1;
    }
  }catch(int num){
    std::cout << "Serial Read Error: failed to read RTCM message of length " << length + 3 << std::endl;
    return "";
  }
  data[0] = last;
  data[1] = cur;
  data[2] = length_buffer;

  std::string ret((unsigned char *) data, (unsigned char *) data + length);
  free(data);
  return ret; // already found an RTCM message, return
}


Observation build_rtk_correction(serialib *serial) {
  std::string rtk_correction = read_raw_rtcm(serial);

  RTKCorrection *correction = new RTKCorrection();
  correction->set_rtk_correction(rtk_correction);

  Observation obs;
  obs.set_allocated_rtk_correction(correction);
  return obs;
}

// We want to produce a series of dummy Communcation values to the TX queue
int main() {
  // Open the message queue, return if it fails
  mqd_t tx_queue = BajaIPC::open_queue(StationIPC::RTK_CORRECTOR_TX_QUEUE, false);
  if (tx_queue == -1) {
    std::cout << "Failed to get transmit queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  // Open the serial port, return if it fails
  serialib serial;
  char port_error = serial.openDevice(SERIAL_PORT, 115200);
  // If connection fails, return the error code otherwise, display a success message
  if (port_error != 1) {
    std::cout << "Error Opening serial port " << SERIAL_PORT << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Successful connection to " << SERIAL_PORT << std::endl;

  while (true) {
    // Should be a blocking call which retrieves an Observation with an RTK correction
    std::string payload = build_rtk_correction(&serial).SerializeAsString();

    int err = BajaIPC::send_message(tx_queue, payload);
    if (err == BajaIPC::QUEUE_FULL) {
      std::cout << "Queue is full, dequeing before enqueing" << std::endl;
      BajaIPC::get_message(tx_queue);
      err = BajaIPC::send_message(tx_queue, payload);
    }

    if (err == BajaIPC::SEND_ERROR) {
      std::cerr << "Could not send RTK correction" << std::endl;
    }
  }
}