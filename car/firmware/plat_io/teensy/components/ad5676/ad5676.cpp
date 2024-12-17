#include "ad5676.hpp"

/*******************************************************************************
 * @brief Initializes the AD5676.
 *
 * @param t_cs_pin   - The chip select pin for the AD5676 device.
 * @param t_ldac_pin - The LDAC pin for the AD5676 device.
 * @param t_rst_pin  - The RST pin for the AD5676 device.
 * @param t_spi_host - The Arduino SPI Host instance/bus that the AD5676 device is on.
 *******************************************************************************/
void AD5676::init(const int8_t t_cs_pin, const int8_t t_ldac_pin, const int8_t t_rst_pin, SPIClass* t_spi_host)
{
    cs_pin_   = t_cs_pin;
    ldac_pin_ = t_ldac_pin;
    rst_pin_  = t_rst_pin;
    spi_host_ = t_spi_host;

    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);

    pinMode(ldac_pin_, OUTPUT);
    digitalWrite(ldac_pin_, HIGH);

    pinMode(rst_pin_, OUTPUT);
    digitalWrite(rst_pin_, HIGH);
}

/*******************************************************************************
 * @brief Sends an SPI transaction to the AD5676 device.
 *
 * @param t_command      - The type of command to send to the AD5676.
 * @param t_chan_addr_id - The channel address of the desired DAC channel (0 - 7).
 * @param t_data         - The data to be sent to the AD5676 device.
 *******************************************************************************/
void AD5676::transfer(const ad5676_command_t t_command, const int8_t t_chan_addr_id,
                      const std::array<uint8_t, 2>& t_data)
{
    std::array<uint8_t, 3> buf;
    buf[0] = (t_command << 4) | (t_chan_addr_id & 0x0F);
    buf[1] = t_data[0];
    buf[2] = t_data[1];
    std::array<uint8_t, 3> ret_buf;

    spi_host_->beginTransaction(spi_settings_);

    digitalWrite(cs_pin_, LOW);
    spi_host_->transfer(buf.data(), ret_buf.data(), 3);

    spi_host_->endTransaction();

    digitalWrite(cs_pin_, HIGH);
}

/*******************************************************************************
 * @brief Sets the level for one of the DAC channels on the AD5676.
 *
 * @param t_chan_addr_id  - The channel address of the desired DAC channel (0 - 7).
 * @param t_new_dac_level - The new value for the DAC channel.
 *******************************************************************************/
void AD5676::setLevel(const int8_t t_chan_addr_id, const uint16_t t_new_dac_level)
{
    std::array<uint8_t, 2> t_data;
    t_data[0] = (t_new_dac_level & 0xFF00) >> 8;
    t_data[1] = (t_new_dac_level & 0x00FF);
    transfer(AD5676_COM_DAC_REG_WRITE_UPDATE, t_chan_addr_id, t_data);
}
