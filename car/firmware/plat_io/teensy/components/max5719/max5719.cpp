#include "max5719.hpp"

/*******************************************************************************
 * @brief Initializes the MAX5719.
 *
 * @param t_cs_pin   - The chip select pin for the MAX5719 device.
 * @param t_ldac_pin - The LDAC pin for the MAX5719 device.
 * @param t_spi_host - The Arduino SPI Host instance/bus that the MAX5719 device is on.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
void MAX5719::init(int8_t t_cs_pin, int8_t t_ldac_pin, SPIClass* t_spi_host)
{
    cs_pin_   = t_cs_pin;
    spi_host_ = t_spi_host;
    ldac_pin_ = t_ldac_pin;

    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);

    pinMode(ldac_pin_, OUTPUT);
    digitalWrite(ldac_pin_, HIGH);
}

/*******************************************************************************
 * @brief Sets and loads the DAC with a 20-bit value.
 *
 * @param t_dac_new_level   - The 20-bit value to be loaded onto the DAC register.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
void MAX5719::setLevel(uint32_t t_dac_new_level)
{
    std::array<uint8_t, 3> buf;
    buf[0] = (t_dac_new_level & 0x000F0000) >> 16;
    buf[1] = (t_dac_new_level & 0x0000FF00) >> 8;
    buf[2] = t_dac_new_level & 0x000000FF;
    std::array<uint8_t, 3> ret_buf;

    digitalWrite(cs_pin_, LOW);
    spi_host_->beginTransaction(spi_settings_);

    spi_host_->transfer(buf.data(), ret_buf.data(), 3);

    spi_host_->endTransaction();

    digitalWrite(cs_pin_, HIGH);
    digitalWrite(ldac_pin_, LOW);
    digitalWrite(ldac_pin_, HIGH);
}
