#ifndef XBEE_BAJA_SERIAL_CONFIGS_H
#define XBEE_BAJA_SERIAL_CONFIGS_H

#include <string>

namespace XbeeBajaSerialConfig {
  static const int BAUDRATE = 921600;
  static const std::string STATION_DEVICE = "/dev/ttyS0";
  static const std::string CAR_DEVICE = "/dev/ttyAMA0";

  static const int CONGESTION_CONTROL_WINDOW = 5;
};

#endif