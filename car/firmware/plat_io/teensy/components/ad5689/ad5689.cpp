#include "ad5689.hpp"

/*******************************************************************************
 * @brief Initializes the AD5689.
 * 
 * @param ad5689_init_params_t - pins and SPI host
 *******************************************************************************/
void AD5689::init(ad5689_init_params_t init_params) {
    params = init_params;

    pinMode(init_params.t_sync_pin, OUTPUT);
    digitalWrite(init_params.t_sync_pin, HIGH);

    pinMode(init_params.t_ldac_pin, OUTPUT);
    digitalWrite(init_params.t_ldac_pin, HIGH);

    pinMode(init_params.t_rst_pin, OUTPUT);
    digitalWrite(init_params.t_rst_pin, HIGH);
}

/*******************************************************************************
 * @brief Sends an SPI transaction to the AD5689 device.
 *
 * @param t_command   - The type of command to send to the AD5689.
 * @param t_chan_mode - The desired DAC channel (A, B, BOTH).
 * @param t_data      - The data to be sent to the AD5689 device.
 *******************************************************************************/
void AD5689::transfer(const t_ad5689_command T_command, const ad5689_channel_t t_chan_mode, const std::array<uint8_t, 2>& t_data) {
    std::array<uint8_t, 4> buf;
    buf[0] = (t_command << 4) | t_chan_mode;
    buf[1] = t_data[0];
    buf[2] = t_data[1];
    std::array<uint8_t, 4> ret_buf;

    params.t_spi_host->beginTransaction(spi_settings_);

    digitalWrite(params.t_sync_pin, LOW);
    params.t_spi_host->transfer(buf.data(), ret_buf.data(), 4);
    params.t_spi_host->endTransaction();
    digitalWrite(params.t_sync_pin, HIGH);

    digitalWrite(params.t_ldac_pin, LOW);
    digitalWrite(params.t_ldac_pin, HIGH);
}

/*******************************************************************************
 * @brief Sets the level for one of the DAC channels on the AD5689.
 *
 * @param t_chan_addr_id  - The desired DAC channel (A, B, BOTH).
 * @param t_new_dac_level - The new value for the DAC channel.
 *******************************************************************************/
void AD5689::setLevel(const ad5689_channel_t t_chan_mode, const uint16_t t_new_dac_level) {
    std::array<uint8_t, 2> t_data;
    t_data[0] = (t_new_dac_level & 0xFF00) >> 8;
    t_data[1] = (t_new_dac_level & 0x00FF);
    transfer(WRITE_DAC_REG, t_chan_mode, t_data);
}