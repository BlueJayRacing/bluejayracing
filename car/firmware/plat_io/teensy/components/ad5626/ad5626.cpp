#include "ad5626.hpp"

/***************************************************************************//**
* @brief Initializes the AD5626.
*
* @param t_cs_pin   - The chip select pin for the AD5626 device.
* @param t_ldac_pin - The LDAC pin for the AD5626 device.
* @param t_clr_pin  - The clear/reset pin for the AD5626 device.
* @param t_spi_host - The Arduino SPI Host instance/bus that the AD5626 device is on.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
void AD5626::init(int8_t t_cs_pin, int8_t t_ldac_pin, int8_t t_clr_pin, SPIClass* t_spi_host)
{
    m_cs_pin = t_cs_pin;
    m_spi_host = t_spi_host;
    m_ldac_pin = t_ldac_pin;
    m_clr_pin = t_clr_pin;

    pinMode(m_cs_pin, OUTPUT);
    digitalWrite(m_cs_pin, HIGH);

    pinMode(m_ldac_pin, OUTPUT);
    digitalWrite(m_ldac_pin, HIGH);

    if (m_clr_pin > 0)
    {
        pinMode(m_clr_pin, OUTPUT);
        digitalWrite(m_clr_pin, HIGH);
    }
}

/***************************************************************************//**
* @brief Sets and loads the DAC with a 12-bit value.
*
* @param t_dac_new_level   - The 12-bit value to be loaded onto the DAC register.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
void AD5626::setLevel(uint16_t t_dac_new_level)
{
    uint8_t buf[2];
    buf[0] = (t_dac_new_level & 0x0F00) >> 8;
    buf[1] = t_dac_new_level & 0x00FF;
    uint8_t ret_buf[2];

    SPISettings settings(1000000, MSBFIRST, SPI_MODE3);

    digitalWrite(m_cs_pin, LOW);
    m_spi_host->beginTransaction(settings);

    m_spi_host->transfer((void*) buf, (void*) ret_buf, 2);

    m_spi_host->endTransaction();
    digitalWrite(m_cs_pin, HIGH);
    digitalWrite(m_ldac_pin, LOW);
    digitalWrite(m_ldac_pin, HIGH);
}

/***************************************************************************//**
* @brief Resets the DAC level to 0.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
void AD5626::resetLevel(void)
{
    if (m_clr_pin >= 0)
    {
        digitalWrite(m_clr_pin, LOW);
        digitalWrite(m_clr_pin, HIGH);
    }
}