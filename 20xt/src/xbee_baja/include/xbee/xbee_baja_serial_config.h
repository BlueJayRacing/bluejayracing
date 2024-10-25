#ifndef XBEE_BAJA_SERIAL_CONFIGS_H
#define XBEE_BAJA_SERIAL_CONFIGS_H

#include <string>

namespace XbeeBajaSerialConfig
{
static const int FACTORY_BAUDRATE       = 9600;
static const int BAJA_BAUDRATE          = 921600; // ours
static const std::string STATION_DEVICE = "/dev/ttyAMA0";
static const std::string CAR_DEVICE     = "/dev/ttyAMA0";

static const int CONGESTION_CONTROL_WINDOW = 5;

static const std::string XBEE_PROFILE_PATH =
    "/home/tanne/bluejayracing/20xt/src/xbee_baja/xctu_configs/baja_profile_5.18.2024.xpro";
}; // namespace XbeeBajaSerialConfig

#endif