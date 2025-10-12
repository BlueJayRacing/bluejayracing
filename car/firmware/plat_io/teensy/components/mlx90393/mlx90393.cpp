#include "mlx90393.hpp"

Magnetometer::Magnetometer() {}

bool Magnetometer::begin(TwoWire& wire, uint8_t addr) {
    wire.begin();
    mlx.begin(0, 0, -1, wire);

    // mlx.setDigitalFiltering(2);
    mlx.setGainSel(1);
    mlx.setOverSampling(0);
    // uint8_t flag = mlx.setBurstSel(0);

    mlx.startBurst(7);

    // sensor.setGain(MLX90393_GAIN_1X);
    // sensor.setOversampling(MLX90393_OSR_0);
    // sensor.setFilter(MLX90393_FILTER_2);

    return true;
}

void Magnetometer::readRawMag(float &x, float &y, float &z) {
    mlx.readData(data);
    x = data.x;
    y = data.y;
    z = data.z;
}