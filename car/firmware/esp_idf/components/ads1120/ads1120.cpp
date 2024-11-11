#include "esp_log.h"

#include "ADS1120.hpp"

static const char* TAG = "ads1120";

#define ADS_SPI_LOCK_TIMEOUT       10
#define ADS_SPI_CLOCK_SPEED_HZ     1 * 1000 * 1000
#define ADS_SPI_MODE               1
#define ADS_CS_EN_PRE_WAIT_CYCLES  2
#define ADS_CS_EN_POST_WAIT_CYCLES 0
#define ADS_SPI_INPUT_DELAY_NS     0

ADS1120::ADS1120() {}

/*******************************************************************************
 * @brief Sends a command to the ADS1120.
 *
 * @param t_command  - 8-bit command to be sent.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::sendCommand(uint8_t t_command)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.flags      = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = t_command;
    t.length     = 1 * 8;

    ret = spi_device_polling_transmit(spi_dev_, &t); // Transmit!
    if (ret != ESP_OK) {
        return ret;
    }

    return ret;
}

/*******************************************************************************
 * @brief writes a register onto the ADS1120.
 *
 * @param t_addr  - The register address.
 * @param t_value - The register value.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::writeRegister(uint8_t t_addr, uint8_t t_value)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.flags      = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = (ADS1120_CMD_WREG | (t_addr << 2));
    t.tx_data[1] = t_value;
    t.length     = 2 * 8; // 2 bytes

    ret = spi_device_polling_transmit(spi_dev_, &t); // Transmit!
    if (ret != ESP_OK) {
        return ret;
    }

    return ret;
}

/*******************************************************************************
 * @brief Reads a register from the ADS1120.
 *
 * @param t_addr  - The register address.
 * @param t_value - A pointer to where the read data will be stored.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::readRegister(uint8_t t_addr, uint8_t* t_data)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.flags      = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length     = 2 * 8; // 2 bytes
    t.rxlength   = 2 * 8; // 2 bytes
    t.tx_data[0] = (ADS1120_CMD_RREG | (t_addr << 2));
    t.tx_data[1] = ADS1120_SPI_MASTER_DUMMY;

    ret = spi_device_polling_transmit(spi_dev_, &t); // Transmit!
    if (ret != ESP_OK) {
        return ret;
    }

    *t_data = t.rx_data[1];
    return ret;
}

/*******************************************************************************
 * @brief Initializes the ADS1120.
 *
 * @param t_cs_pin   - The chip select pin.
 * @param t_drdy_pin - The data ready pin.
 * @param t_spi_host - The SPI host/bus that the device is on.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::init(gpio_num_t t_cs_pin, gpio_num_t t_drdy_pin, spi_host_device_t t_spi_host)
{
    // Set pins up
    drdy_pin_ = t_drdy_pin;

    gpio_set_direction(drdy_pin_, GPIO_MODE_INPUT);

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(spi_device_interface_config_t));

    devcfg.clock_source     = SPI_CLK_SRC_DEFAULT;
    devcfg.mode             = ADS_SPI_MODE;
    devcfg.cs_ena_pretrans  = ADS_CS_EN_PRE_WAIT_CYCLES;
    devcfg.cs_ena_posttrans = ADS_CS_EN_POST_WAIT_CYCLES;
    devcfg.clock_speed_hz   = ADS_SPI_CLOCK_SPEED_HZ;
    devcfg.input_delay_ns   = ADS_SPI_INPUT_DELAY_NS;
    devcfg.spics_io_num     = t_cs_pin; // CS pin
    devcfg.flags            = 0;
    devcfg.queue_size       = 1;

    esp_err_t err = spi_bus_add_device(t_spi_host, &devcfg, &spi_dev_);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Added SPI Device to Bus\n");

    vTaskDelay(5);

    reset();
    vTaskDelay(5);
    return startSync(); // Send start/sync for continuous conversion mode
}

/*******************************************************************************
 * @brief Checks if the data on the ADC is ready to be read.
 *
 * @return Returns true if ready, and false if not.
 *******************************************************************************/
bool ADS1120::isDataReady() { return !gpio_get_level(drdy_pin_); }

/*******************************************************************************
 * @brief Reads the data value from the ADC and stores it in t_data.
 * 
 * @param t_data A pointer to where the ADC value is stored.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::readADC(uint16_t* t_data)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    memset(t.tx_data, ADS1120_SPI_MASTER_DUMMY, 2);

    t.length   = 2 * 8; // 2 bytes
    t.rxlength = 2 * 8; // 2 bytes

    ret = spi_device_polling_transmit(spi_dev_, &t); // Transmit!
    if (ret != ESP_OK) {
        return ret;
    }

    *t_data = t.rx_data[0];
    *t_data = (*t_data << 8) | t.rx_data[1];
    return ret;
}

/*******************************************************************************
 * @brief Writes a masked register onto the ADS1120.
 *
 * @param t_value    - The new register value.
 * @param t_mask     - The register mask.
 * @param t_address  - The register address.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::writeRegisterMasked(uint8_t t_value, uint8_t t_mask, uint8_t t_address)
{
    uint8_t register_contents;
    readRegister(t_address, &register_contents);

    register_contents = register_contents & ~t_mask;

    register_contents = register_contents | t_value;

    return writeRegister(t_address, register_contents);
}

/*******************************************************************************
 * @brief Set the multiplexer on the ADS1120.
 *
 * @param t_value    - The new multiplexer value.
 *
 * @return Returns 0 for success or negative error code.
 *
 * @note | Value | AINp | AINn |
 * @note |-------|------|------|
 * @note | 0x00  | AIN0 | AIN1 |
 * @note | 0X01  | AIN0 | AIN2 |
 * @note | 0X02  | AIN0 | AIN3 |
 * @note | 0X03  | AIN1 | AIN2 |
 * @note | 0X04  | AIN1 | AIN3 |
 * @note | 0X05  | AIN2 | AIN3 |
 * @note | 0X06  | AIN1 | AIN0 |
 * @note | 0X07  | AIN3 | AIN2 |
 * @note | 0X08  | AIN0 | AVSS |
 * @note | 0X09  | AIN1 | AVSS |
 * @note | 0X0A  | AIN2 | AVSS |
 * @note | 0X0B  | AIN3 | AVSS |
 * @note | 0X0C  |  REF/4 MON  |
 * @note | 0X0D  | APWR/4 MON  |
 * @note | 0X0E  |   SHORTED   |
 *******************************************************************************/
esp_err_t ADS1120::setMultiplexer(uint8_t t_value)
{
    if (t_value > 0x0E) {
        t_value = 0x00;
    }
    t_value = t_value << 4; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_MUX, ADS1120_CONFIG_REG0_ADDRESS);
}

/*******************************************************************************
 * @brief Set the gain on the ADS1120.
 *
 * @param t_value - The new gain value.
 *
 * @return Returns 0 for success or negative error code.
 *
 * @note Possible gain values are 1, 2, 4, 8, 16, 32, 64, 128.
 *******************************************************************************/
esp_err_t ADS1120::setGain(uint8_t t_gain)
{
    uint8_t value = 0x00;
    switch (t_gain) {
    case 1:
        value = 0x00;
        break;
    case 2:
        value = 0x01;
        break;
    case 4:
        value = 0x02;
        break;
    case 8:
        value = 0x03;
        break;
    case 16:
        value = 0x04;
        break;
    case 32:
        value = 0x05;
        break;
    case 64:
        value = 0x06;
        break;
    case 128:
        value = 0x07;
        break;
    default:
        value = 0x00;
        break;
    }
    value = value << 1; // Shift to match with mask
    return writeRegisterMasked(value, ADS1120_REG_MASK_GAIN, ADS1120_CONFIG_REG0_ADDRESS);
}

/*******************************************************************************
 * @brief Set the PGA bypass on the ADS1120.
 *
 * @param t_value - The new PGA bypass value. Bypasses the PGA if true.
 *
 * @return Returns 0 for success or negative error code.
 *
 * @note PGA can only be disabled for gains 1, 2, 4.
 *******************************************************************************/
esp_err_t ADS1120::setPGAbypass(bool t_value)
{
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_PGA_BYPASS, ADS1120_CONFIG_REG0_ADDRESS);
}

/*******************************************************************************
 * @brief Set the data rate of the ADS1120.
 *
 * @param t_value - The new data rate value.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::setDataRate(uint8_t t_value)
{
    /* Sets the data rate for the ADC. See table 18 in datasheet for datarates
       in various operating modes. */
    if (t_value > 0x07) {
        t_value = 0x00;
    }
    t_value = t_value << 5; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_DATARATE, ADS1120_CONFIG_REG1_ADDRESS);
}

/*******************************************************************************
 * @brief Set the operational mode of the ADS1120.
 *
 * @param t_value - The new op mode value.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::setOpMode(uint8_t t_value)
{
    /*
       0 - Normal mode
       1 - Duty-cycle mode
       2 - Turbo mode
    */
    if (t_value > 0x02) {
        t_value = 0x00;
    }
    t_value = t_value << 3; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_OP_MODE, ADS1120_CONFIG_REG1_ADDRESS);
}

/*******************************************************************************
 * @brief Set the conversion mode of the ADS1120.
 *
 * @param t_value - The new conversion mode value.
 *
 * @return Returns 0 for success or negative error code.
 *
 * @note 0 - Single shot mode
 * @note 1 - Continuous conversion mode
 *******************************************************************************/
esp_err_t ADS1120::setConversionMode(uint8_t t_value)
{
    if (t_value > 0x01) {
        t_value = 0x00;
    }
    t_value = t_value << 2; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_CONV_MODE, ADS1120_CONFIG_REG1_ADDRESS);
}

/*******************************************************************************
 * @brief Set the temperature mode of the ADS1120.
 *
 * @param t_value - The new temperature mode value.
 *
 * @return Returns 0 for success or negative error code.
 *
 * @note 0 - Disables temperature sensor
 * @note 1 - Enables temperature sensor
 *******************************************************************************/
esp_err_t ADS1120::setTemperatureMode(uint8_t t_value)
{
    if (t_value > 0x01) {
        t_value = 0x00;
    }
    t_value = t_value << 1; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_TEMP_MODE, ADS1120_CONFIG_REG1_ADDRESS);
}

/*******************************************************************************
 * @brief Turns on/off the current burn-out sources of the ADS1120.
 *
 * @param t_value - True for on, false for off.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::setBurnoutCurrentSources(bool t_value)
{
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_BURNOUT_SOURCES, ADS1120_CONFIG_REG1_ADDRESS);
}

/*******************************************************************************
 * @brief Sets the voltage reference of the ADS1120.
 *
 * @param t_value - The new voltage reference value.
 *
 * @return Returns 0 for success or negative error code.
 * @note 0 - Internal 2.048 V
 * @note 1 - External on REFP0 and REFN0 inputs
 * @note 2 - External on AIN0/REFP1 and AIN3/REFN1 inputs
 * @note 3 - Use analog supply as reference
 *******************************************************************************/
esp_err_t ADS1120::setVoltageRef(uint8_t t_value)
{
    if (t_value > 0x03) {
        t_value = 0x00;
    }
    t_value = t_value << 6; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_VOLTAGE_REF, ADS1120_CONFIG_REG2_ADDRESS);
}

/*******************************************************************************
 * @brief Sets the filter for the ADS1120.
 *
 * @param t_value - The new FIR value.
 *
 * @return Returns 0 for success or negative error code.
 * @note 0 - No 50 or 60 Hz rejection
 * @note 1 - Both 50 and 60 Hz rejection
 * @note 2 - 50 Hz rejection
 * @note 3 - 60 Hz rejection
 *******************************************************************************/
esp_err_t ADS1120::setFIR(uint8_t t_value)
{
    if (t_value > 0x03) {
        t_value = 0x00;
    }
    t_value = t_value << 4; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_FIR_CONF, ADS1120_CONFIG_REG2_ADDRESS);
}

/*******************************************************************************
 * @brief Configures the low-side power switch between AIN3/REFN1 and AVSS.
 *
 * @param t_value - new power switch value.
 *
 * @return Returns 0 for success or negative error code.
 * @note 0 - Always open
 * @note 1 - Automatically closes when START/SYNC command is sent and opens when
 * @note     POWERDOWN command is issues.
 *******************************************************************************/
esp_err_t ADS1120::setPowerSwitch(uint8_t t_value)
{
    if (t_value > 0x01) {
        t_value = 0x00;
    }
    t_value = t_value << 3; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_PWR_SWITCH, ADS1120_CONFIG_REG2_ADDRESS);
}

/*******************************************************************************
 * @brief Sets current for both IDAC1 and IDAC2 excitation sources.
 *
 * @param t_value - new IDAC current value.
 *
 * @return Returns 0 for success or negative error code.
 * @note 0 - Off
 * @note 1 - 10 uA
 * @note 2 - 50 uA
 * @note 3 - 100 uA
 * @note 4 - 250 uA
 * @note 5 - 500 uA
 * @note 6 - 1000 uA
 * @note 7 - 1500 uA
 *******************************************************************************/
esp_err_t ADS1120::setIDACcurrent(uint8_t t_value)
{
    if (t_value > 0x07) {
        t_value = 0x00;
    }
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_IDAC_CURRENT, ADS1120_CONFIG_REG2_ADDRESS);
}

/*******************************************************************************
 * @brief Selects where IDAC1 is routed to.
 *
 * @param t_value - new IDAC1 routing value.
 *
 * @return Returns 0 for success or negative error code.
 *
 * @note 0 - Disabled
 * @note 1 - AIN0/REFP1
 * @note 2 - AIN1
 * @note 3 - AIN2
 * @note 4 - AIN3/REFN1
 * @note 5 - REFP0
 * @note 6 - REFN0
 *******************************************************************************/
esp_err_t ADS1120::setIDAC1routing(uint8_t t_value)
{
    if (t_value > 0x06) {
        t_value = 0x00;
    }
    t_value = t_value << 5; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_IDAC1_ROUTING, ADS1120_CONFIG_REG3_ADDRESS);
}

/*******************************************************************************
 * @brief Selects where IDAC2 is routed to.
 *
 * @param t_value - new IDAC2 routing value.
 *
 * @return Returns 0 for success or negative error code.
 *
 * @note 0 - Disabled
 * @note 1 - AIN0/REFP1
 * @note 2 - AIN1
 * @note 3 - AIN2
 * @note 4 - AIN3/REFN1
 * @note 5 - REFP0
 * @note 6 - REFN0
 *******************************************************************************/
esp_err_t ADS1120::setIDAC2routing(uint8_t t_value)
{
    if (t_value > 0x06) {
        t_value = 0x00;
    }
    t_value = t_value << 2; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_IDAC2_ROUTING, ADS1120_CONFIG_REG3_ADDRESS);
}

/*******************************************************************************
 * @brief Sets the DRDY mode for the device on continuous mode.
 *
 * @param t_value - new DRDY mode value.
 *
 * @return Returns 0 for success or negative error code.
 *
 * @note 0 - Only the dedicated DRDY pin is used  (Default)
 * @note 1 - Data ready indicated on DOUT/DRDY and DRDY
 *******************************************************************************/
esp_err_t ADS1120::setDRDYmode(uint8_t t_value)
{
    if (t_value > 0x01) {
        t_value = 0x00;
    }
    t_value = t_value << 1; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_DRDY_MODE, ADS1120_CONFIG_REG3_ADDRESS);
}

/*******************************************************************************
 * @brief Resets the ADS1120.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::reset() { return sendCommand(ADS1120_CMD_RESET); }

/*******************************************************************************
 * @brief Start syncing with the ADS1120.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::startSync() { return sendCommand(ADS1120_CMD_START_SYNC); }

/*******************************************************************************
 * @brief Powers down the ADS1120.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t ADS1120::powerDown() { return sendCommand(ADS1120_CMD_PWRDWN); }

esp_err_t ADS1120::rdata() { return sendCommand(ADS1120_CMD_RDATA); }