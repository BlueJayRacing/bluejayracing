#include "ad5626.hpp"

/*******************************************************************************
 * @brief Initializes the AD5626.
 *
 * @param t_cs_pin   - The chip select pin for the AD5626 device.
 * @param t_ldac_pin - The LDAC pin for the AD5626 device.
 * @param t_clr_pin  - The clear/reset pin for the AD5626 device. (set to <0 if not used)
 * @param t_spi_host - The Arduino SPI Host instance/bus that the AD5626 device is on.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
void AD5626::init(int8_t t_cs_pin, int8_t t_ldac_pin, int8_t t_clr_pin, SPIClass* t_spi_host)
{
    cs_pin_   = t_cs_pin;
    spi_host_ = t_spi_host;
    ldac_pin_ = t_ldac_pin;
    clr_pin_  = t_clr_pin;

    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);

    pinMode(ldac_pin_, OUTPUT);
    digitalWrite(ldac_pin_, HIGH);

    if (clr_pin_ > 0) {
        pinMode(clr_pin_, OUTPUT);
        digitalWrite(clr_pin_, HIGH);
    }
}

/*******************************************************************************
 * @brief Sets and loads the DAC with a 12-bit value.
 *
 * @param t_dac_new_level   - The 12-bit value to be loaded onto the DAC register.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
void AD5626::setLevel(uint16_t t_dac_new_level)
{
    std::array<uint8_t, 2> buf;
    buf[0] = (t_dac_new_level & 0x0F00) >> 8;
    buf[1] = t_dac_new_level & 0x00FF;
    std::array<uint8_t, 2> ret_buf;

    digitalWrite(cs_pin_, LOW);
    spi_host_->beginTransaction(spi_settings_);

    spi_host_->transfer(buf.data(), ret_buf.data(), 2);

    spi_host_->endTransaction();

    digitalWrite(cs_pin_, HIGH);
    digitalWrite(ldac_pin_, LOW);
    digitalWrite(ldac_pin_, HIGH);
}

/********************************************************************************
 * @brief Resets the DAC level to 0.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
void AD5626::resetLevel(void)
{
    if (clr_pin_ >= 0) {
        digitalWrite(clr_pin_, LOW);
        digitalWrite(clr_pin_, HIGH);
    }
}