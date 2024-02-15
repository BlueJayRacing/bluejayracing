#ifndef XBEE_BAJA_NETWORK_CONFIGS_H
#define XBEE_BAJA_NETWORK_CONFIGS_H

#include <string>
#include <map>

#define XBEE_BAJA_MAX_PAYLOAD_SIZE 100
#define XBEE_BAJA_CM = 0x3FFFFFFFFFFFF;
#define XBEE_BAJA_HP 0
#define XBEE_BAJA_TX 2
#define XBEE_BAJA_BR 1
#define XBEE_BAJA_AP 1
#define XBEE_BAJA_ID 2015
#define XBEE_BAJA_MT 0

namespace XbeeBajaSerialConfig {
  static const int BAUDRATE = 921600;
  static const char* STATION_DEVICE = "/dev/ttyUSB0";
  static const char* CAR_DEVICE = "PUT CAR DEVICE HERE";
};

#endif