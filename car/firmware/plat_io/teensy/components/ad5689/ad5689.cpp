#include "ad5689.hpp"

/*******************************************************************************
 * @brief Initializes the AD5689.
 * 
 * @param ad5689_init_params_t - pins and SPI host
 *******************************************************************************/
void AD5689::init(ad5689_init_params_t init_params) {
    params = init_params;
    // spi_host_ = s;

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
void AD5689::transfer(const t_ad5689_command t_command, const ad5689_channel_t t_chan_mode, const std::array<uint8_t, 2>& t_data) {
    std::array<uint8_t, 3> buf;
    buf[0] = (t_command << 4) | t_chan_mode;     // same as buf[0] = 0x31;
    // buf[0] = 0b00110001; // same as buf[0] = 49;
    buf[1] = t_data[0];
    buf[2] = t_data[1];
    std::array<uint8_t, 3> ret_buf;
    SPI.begin();
    digitalWrite(params.t_sync_pin, LOW);
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));
    Serial.println("Begun Transaction");

    SPI.transfer(buf.data(), ret_buf.data(), 3);
    Serial.println("About to end Transaction");
    SPI.endTransaction();
    Serial.println("Transaction finished");
    digitalWrite(params.t_sync_pin, HIGH);

    digitalWrite(params.t_ldac_pin, LOW);
    digitalWrite(params.t_ldac_pin, HIGH);
}

/*******************************************************************************
 * @brief Sets the level for one of the DAC channels on the AD5689.
 *
 * @param t_chan_id  - The desired DAC channel (A, B, BOTH).
 * @param t_new_dac_level - The new value for the DAC channel.
 *******************************************************************************/
void AD5689::setLevel(const ad5689_channel_t t_chan_mode, const uint16_t t_new_dac_level) {
    std::array<uint8_t, 2> t_data;
    Serial.println("Starting Set Level");
    t_data[0] = (t_new_dac_level & 0xFF00) >> 8;
    t_data[1] = (t_new_dac_level & 0x00FF);
    transfer(WRITE_DAC_REG, t_chan_mode, t_data);
}