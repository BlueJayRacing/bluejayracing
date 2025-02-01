#include "ad5689r.hpp"

/*******************************************************************************
 * @brief Initializes the AD5689.
 * 
 * @param init_params - pins and SPI host
 *******************************************************************************/
void AD5689R::init(ad5689r_init_param_t init_params) {
    bus_params_ = init_params;

    pinMode(bus_params_.cs_pin, OUTPUT);
    digitalWrite(bus_params_.cs_pin, HIGH);

    pinMode(bus_params_.ldac_pin, OUTPUT);
    digitalWrite(bus_params_.ldac_pin, HIGH);

    pinMode(bus_params_.clr_pin, OUTPUT);
    digitalWrite(bus_params_.clr_pin, HIGH);
}

/*******************************************************************************
 * @brief Sends an SPI transaction to the AD5689 device.
 *
 * @param t_command   - The type of command to send to the AD5689.
 * @param t_chan_mode - The desired DAC channel (A, B, BOTH).
 * @param t_data      - The data to be sent to the AD5689 device.
 *******************************************************************************/
void AD5689R::transfer(const ad5689r_command_t t_command, const ad5689r_channel_t t_chan_mode, const std::array<uint8_t, 2>& t_data) {
    std::array<uint8_t, 3> buf;
    buf[0] = (t_command << 4) | t_chan_mode;
    buf[1] = t_data[0];
    buf[2] = t_data[1];
    std::array<uint8_t, 3> ret_buf;

    digitalWrite(bus_params_.cs_pin, LOW);
    bus_params_.spi_host->beginTransaction(spi_settings_);
    bus_params_.spi_host->transfer(buf.data(), ret_buf.data(), 3);
    bus_params_.spi_host->endTransaction();
    digitalWrite(bus_params_.cs_pin, HIGH);

    digitalWrite(bus_params_.ldac_pin, LOW);
    digitalWrite(bus_params_.ldac_pin, HIGH);
}

/*******************************************************************************
 * @brief Sets the level for one of the DAC channels on the AD5689.
 *
 * @param t_chan_id  - The desired DAC channel (A, B, BOTH).
 * @param t_new_dac_level - The new value for the DAC channel.
 *******************************************************************************/
void AD5689R::setLevel(const ad5689r_channel_t t_chan_mode, const uint16_t t_new_dac_level) {
    std::array<uint8_t, 2> t_data;
    t_data[0] = (t_new_dac_level & 0xFF00) >> 8;
    t_data[1] = (t_new_dac_level & 0x00FF);
    transfer(WRITE_DAC_REG, t_chan_mode, t_data);
}