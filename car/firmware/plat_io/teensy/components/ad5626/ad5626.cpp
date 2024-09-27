#include "ad5626.hpp"

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

void AD5626::resetLevel(void)
{
    if (m_clr_pin >= 0)
    {
        digitalWrite(m_clr_pin, LOW);
        delay(1);
        digitalWrite(m_clr_pin, HIGH);
    }
}