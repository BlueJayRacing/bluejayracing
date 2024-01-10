#ifndef XBEE_BAJA_NETWORK_CONFIGS_H
#define XBEE_BAJA_NETWORK_CONFIGS_H

  namespace XbeeBajaNetworkConfig {
    // TODO: Break these configs into serial & network config headers
    struct cmd {
    const char* name;
    const int value;
    };

    static const long XBEE_BAJA_CM = 0x3FFFFFFFFFFFF; // Channel mask

    static const struct cmd XBEE_BAJA_CONFIGS[] = {
    {"HP", 0},
    {"TX", 2},
    {"BR", 1},
    {"AP", 1},
    {"ID", 2015},
    {"MT", 0},
    {"NP", 100},
  };
}

#endif