#ifndef STATION_SERIAL_XBEE_CONFIG_H
#define STATION_SERIAL_XBEE_CONFIG_H


// TODO: Turn this into a portable xbee_serial_config.h instead
  namespace StationSerialXbeeConfig {
    static const int XBEE_BAJA_BAUDRATE = 921600;
    static const int XBEE_BAJA_PARITY = 0;
    static const int XBEE_BAJA_STOPBITS = 1;
    static const std::string LINUX_SERIAL_DEVICE_ID = "/dev/ttyS0";

    struct cmd {
    const char* name;
    const int value;
    };

    static const struct cmd XBEE_BAJA_CONFIGS[] = {
    {"BD", XBEE_BAJA_BAUDRATE},
    {"NB", XBEE_BAJA_PARITY},
    {"SB", XBEE_BAJA_STOPBITS},
  };
}

#endif