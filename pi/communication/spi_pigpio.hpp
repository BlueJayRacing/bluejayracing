#ifndef SPI_PIGPIO_HPP
#define SPI_PIGPIO_HPP

#include "spi_d.hpp"
#include <cstdint>
#include <linux/spi/spidev.h>
#include <pigpiod_if2.h>
#include <vector>

int spi_write(spi_d sd, std::uint8_t reg, std::uint8_t data);

std::uint8_t spi_read(spi_d sd, std::uint8_t reg);

std::vector<std::uint8_t> spi_bulk_read(spi_d sd, std::uint8_t reg, int len);

#endif