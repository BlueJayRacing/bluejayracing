#include <rclcpp/rclcpp.hpp>
#include <baja_msgs/msg/rtk_correction.hpp>
#include "proto/baja_live_comm.pb.h"
#include "serialib.h"

#define BAJA_RTK_SERIAL_PORT "/dev/ttyACM0"

namespace baja_rtk {

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


class RTKCorrectionsPublisher : public rclcpp::Node {
private:
  rclcpp::Publisher<baja_msgs::msg::RTKCorrection>::SharedPtr publisher_;
  serialib serial_;

public:
  RTKCorrectionsPublisher();
  void publish_rtk_corrections();
};

}  // namespace baja_rtk