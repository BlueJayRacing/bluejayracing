#include "esp_log.h"

#include "ADS1120.hpp"

static const char* TAG = "ads1120";

#define ADS_SPI_LOCK_TIMEOUT       10
#define ADS_SPI_CLOCK_SPEED_HZ     4 * 1000 * 1000
#define ADS_SPI_MODE               1
#define ADS_CS_EN_PRE_WAIT_CYCLES  2
#define ADS_CS_EN_POST_WAIT_CYCLES 0
#define ADS_SPI_INPUT_DELAY_NS     0

ADS1120::ADS1120() { memset(&regs_, 0x00, sizeof(ads1120_regs_t)); }

/*******************************************************************************
 * @brief Initializes the ADS1120.
 *
 * @param t_cs_pin   - The chip select pin.
 * @param t_drdy_pin - The data ready pin.
 * @param t_spi_host - The SPI host/bus that the device is on.
 *
 * @return Returns ESP_OK for success or a non-zero value otherwise.
 *******************************************************************************/
esp_err_t ADS1120::init(ads1120_init_param_t t_init_param)
{
    // Set pins up
    drdy_pin_ = t_init_param.drdy_pin;

    gpio_set_direction(drdy_pin_, GPIO_MODE_INPUT);

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(spi_device_interface_config_t));

    devcfg.clock_source     = SPI_CLK_SRC_DEFAULT;
    devcfg.mode             = ADS_SPI_MODE;
    devcfg.cs_ena_pretrans  = ADS_CS_EN_PRE_WAIT_CYCLES;
    devcfg.cs_ena_posttrans = ADS_CS_EN_POST_WAIT_CYCLES;
    devcfg.clock_speed_hz   = ADS_SPI_CLOCK_SPEED_HZ;
    devcfg.input_delay_ns   = ADS_SPI_INPUT_DELAY_NS;
    devcfg.spics_io_num     = t_init_param.cs_pin; // CS pin
    devcfg.flags            = 0;
    devcfg.queue_size       = 1;

    esp_err_t ret = spi_bus_add_device(t_init_param.spi_host, &devcfg, &spi_dev_);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "Added ADS1120 Device to Bus\n");

    vTaskDelay(5);

    ret = reset();
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(5);
    return startSync(); // Send start/sync for continuous conversion mode
}

/*******************************************************************************
 * @brief Configures the ADS1120 with the proper registers. If a new register
 *        is equivalent to an old register, no spi transaction occurs.
 *
 * @param t_nregs   - The new registers to write to the ADC.
 *
 * @return Returns ESP_OK for success or a non-zero value otherwise.
 *******************************************************************************/
esp_err_t ADS1120::configure(ads1120_regs_t t_nregs)
{
    esp_err_t ret;

    if (t_nregs.channels != regs_.channels) {
        ret = writeRegisterMasked(t_nregs.channels << 4, ADS1120_REG_MASK_MUX, ADS1120_CONFIG_REG0_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set analog channels");
            return ret;
        }

        ESP_LOGI(TAG, "Set analog channels w/: %d", t_nregs.channels);
        regs_.channels = t_nregs.channels;
    }

    if (t_nregs.volt_refs != regs_.volt_refs) {
        ret = writeRegisterMasked(t_nregs.volt_refs << 6, ADS1120_REG_MASK_VOLTAGE_REF, ADS1120_CONFIG_REG2_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set voltage references");
            return ret;
        }

        ESP_LOGI(TAG, "Set voltage references w/: %d", t_nregs.volt_refs);
        regs_.volt_refs = t_nregs.volt_refs;
    }

    if (t_nregs.gain != regs_.gain) {
        ret = writeRegisterMasked(t_nregs.gain << 1, ADS1120_REG_MASK_GAIN, ADS1120_CONFIG_REG0_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set gain");
            return ret;
        }

        ESP_LOGI(TAG, "Set gain w/: %d", t_nregs.gain);
        regs_.gain = t_nregs.gain;
    }

    if (t_nregs.pga_bypass != regs_.pga_bypass) {
        ret = writeRegisterMasked(t_nregs.pga_bypass, ADS1120_REG_MASK_PGA_BYPASS, ADS1120_CONFIG_REG0_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set PGA bypass");
            return ret;
        }

        ESP_LOGI(TAG, "Set PGA bypass w/: %d", t_nregs.pga_bypass);
        regs_.pga_bypass = t_nregs.pga_bypass;
    }

    if (t_nregs.data_rate != regs_.data_rate) {
        ret = setDataRate(t_nregs.data_rate);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set data rate");
            return ret;
        }

        ESP_LOGI(TAG, "Set data rate w/: %d", t_nregs.data_rate);
        regs_.data_rate = t_nregs.data_rate;
    }

    if (t_nregs.op_mode != regs_.op_mode) {
        ret = writeRegisterMasked(t_nregs.op_mode << 3, ADS1120_REG_MASK_OP_MODE, ADS1120_CONFIG_REG1_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set op mode");
            return ret;
        }

        ESP_LOGI(TAG, "Set op mode w/: %d", t_nregs.op_mode);
        regs_.op_mode = t_nregs.op_mode;
    }

    if (t_nregs.conv_mode != regs_.conv_mode) {
        ret = writeRegisterMasked(t_nregs.conv_mode << 2, ADS1120_REG_MASK_CONV_MODE, ADS1120_CONFIG_REG1_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set conversion mode");
            return ret;
        }

        ESP_LOGI(TAG, "Set conversion mode w/: %d", t_nregs.conv_mode);
        regs_.conv_mode = t_nregs.conv_mode;
    }

    if (t_nregs.temp_mode != regs_.temp_mode) {
        ret = writeRegisterMasked(t_nregs.temp_mode << 1, ADS1120_REG_MASK_TEMP_MODE, ADS1120_CONFIG_REG1_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set temperature mode");
            return ret;
        }

        ESP_LOGI(TAG, "Set temperature mode w/: %d", t_nregs.temp_mode);
        regs_.temp_mode = t_nregs.temp_mode;
    }

    if (t_nregs.burn_sources != regs_.burn_sources) {
        ret = writeRegisterMasked(t_nregs.burn_sources, ADS1120_REG_MASK_BURNOUT_SOURCES, ADS1120_CONFIG_REG1_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set burnout current sources");
            return ret;
        }

        ESP_LOGI(TAG, "Set burnout current sources w/: %d", t_nregs.burn_sources);
        regs_.burn_sources = t_nregs.burn_sources;
    }

    if (t_nregs.fir != regs_.fir) {
        ret = writeRegisterMasked(t_nregs.fir << 4, ADS1120_REG_MASK_FIR_CONF, ADS1120_CONFIG_REG2_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set fir");
            return ret;
        }

        ESP_LOGI(TAG, "Set fir w/: %d", t_nregs.fir);
        regs_.fir = t_nregs.fir;
    }

    if (t_nregs.power_switch != regs_.power_switch) {
        ret = writeRegisterMasked(t_nregs.power_switch << 3, ADS1120_REG_MASK_PWR_SWITCH, ADS1120_CONFIG_REG2_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set power switch");
            return ret;
        }

        ESP_LOGI(TAG, "Set power switch w/: %d", t_nregs.power_switch);
        regs_.power_switch = t_nregs.power_switch;
    }

    if (t_nregs.idac_current != regs_.idac_current) {
        ret = writeRegisterMasked(t_nregs.idac_current, ADS1120_REG_MASK_IDAC_CURRENT, ADS1120_CONFIG_REG2_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set IDAC current");
            return ret;
        }

        ESP_LOGI(TAG, "Set IDAC current w/: %d", t_nregs.idac_current);
        regs_.idac_current = t_nregs.idac_current;
    }

    if (t_nregs.idac1_routing != regs_.idac1_routing) {
        ret = writeRegisterMasked(t_nregs.idac1_routing << 5, ADS1120_REG_MASK_IDAC1_ROUTING,
                                  ADS1120_CONFIG_REG3_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set IDAC1 routing");
            return ret;
        }

        ESP_LOGI(TAG, "Set IDAC1 routing w/: %d", t_nregs.idac1_routing);
        regs_.idac1_routing = t_nregs.idac1_routing;
    }

    if (t_nregs.idac2_routing != regs_.idac2_routing) {
        ret = writeRegisterMasked(t_nregs.idac2_routing << 2, ADS1120_REG_MASK_IDAC2_ROUTING,
                                  ADS1120_CONFIG_REG3_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set IDAC2 routing");
            return ret;
        }

        ESP_LOGI(TAG, "Set IDAC2 routing w/: %d", t_nregs.idac2_routing);
        regs_.idac2_routing = t_nregs.idac2_routing;
    }

    if (t_nregs.drdy_mode != regs_.drdy_mode) {
        ret = writeRegisterMasked(t_nregs.drdy_mode << 1, ADS1120_REG_MASK_DRDY_MODE, ADS1120_CONFIG_REG3_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set DRDY mode");
            return ret;
        }

        ESP_LOGI(TAG, "Set DRDY mode w/: %d", t_nregs.drdy_mode);
        regs_.drdy_mode = t_nregs.drdy_mode;
    }

    return ESP_OK;
}

/*******************************************************************************
 * @brief Gets the current recorded registers of the ADS1120 instance of the
 *        microcontroller (does not read the ADC).
 *
 * @param regs   - The pointer to where the current registers will be copied to.
 *******************************************************************************/
void ADS1120::getRegs(ads1120_regs_t* regs)
{
    if (regs == NULL) {
        ESP_LOGE(TAG, "Given register pointer is NULL");
        return;
    }
    memcpy(regs, &regs_, sizeof(ads1120_regs_t));
}

/*******************************************************************************
 * @brief Checks if the data on the ADC is ready to be read.
 *
 * @return Returns true if ready, and false if not.
 *******************************************************************************/
bool ADS1120::isDataReady() const { return !gpio_get_level(drdy_pin_); }

/*******************************************************************************
 * @brief Reads the data value from the ADC and stores it in t_data.
 *
 * @param t_data A pointer to where the ADC value is stored.
 *
 * @return Returns ESP_OK for success or a non-zero value otherwise.
 *******************************************************************************/
esp_err_t ADS1120::readADC(int16_t* t_data)
{
    esp_err_t ret;

    if (regs_.conv_mode == SINGLE_SHOT) {
        ret = startSync();
        if (ret) {
            return ret;
        }

        while (!isDataReady()){};
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    memset(t.tx_data, ADS1120_SPI_MASTER_DUMMY, 2);

    t.length   = 2 * 8; // 2 bytes
    t.rxlength = 2 * 8; // 2 bytes

    ret = spi_device_queue_trans(spi_dev_, &t, portMAX_DELAY); // Transmit!
    if (ret != ESP_OK) {
        return ret;
    }

    spi_transaction_t* trans_result;

    ret = spi_device_get_trans_result(spi_dev_, &trans_result, portMAX_DELAY);

    *t_data = trans_result->rx_data[0];
    *t_data = (*t_data << 8) | trans_result->rx_data[1];

    return ESP_OK;
}

/*******************************************************************************
 * @brief Sends a command to the ADS1120.
 *
 * @param t_command  - 8-bit command to be sent.
 *
 * @return Returns ESP_OK for success or a non-zero value otherwise.
 *******************************************************************************/
esp_err_t ADS1120::sendCommand(uint8_t t_command)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.flags      = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = t_command;
    t.length     = 1 * 8;

    return spi_device_polling_transmit(spi_dev_, &t); // Transmit!
}

/*******************************************************************************
 * @brief writes a register onto the ADS1120.
 *
 * @param t_addr  - The register address.
 * @param t_value - The register value.
 *
 * @return Returns ESP_OK for success or a non-zero value otherwise.
 *******************************************************************************/
esp_err_t ADS1120::writeRegister(uint8_t t_addr, uint8_t t_value)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.flags      = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = (ADS1120_CMD_WREG | (t_addr << 2));
    t.tx_data[1] = t_value;
    t.length     = 2 * 8; // 2 bytes

    return spi_device_polling_transmit(spi_dev_, &t); // Transmit!
}

/*******************************************************************************
 * @brief Reads a register from the ADS1120.
 *
 * @param t_addr  - The register address.
 * @param t_value - A pointer to where the read data will be stored.
 *
 * @return Returns ESP_OK for success or a non-zero value otherwise.
 *******************************************************************************/
esp_err_t ADS1120::readRegister(uint8_t t_addr, uint8_t* t_data) const
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

    return ESP_OK;
}

/*******************************************************************************
 * @brief Writes a masked register onto the ADS1120.
 *
 * @param t_value    - The new register value.
 * @param t_mask     - The register mask.
 * @param t_address  - The register address.
 *
 * @return Returns ESP_OK for success or a non-zero value otherwise.
 *******************************************************************************/
esp_err_t ADS1120::writeRegisterMasked(uint8_t t_value, uint8_t t_mask, uint8_t t_address)
{
    uint8_t register_contents;
    esp_err_t ret = readRegister(t_address, &register_contents);
    if (ret != ESP_OK) {
        return ret;
    }

    register_contents = register_contents & ~t_mask;

    register_contents = register_contents | t_value;

    return writeRegister(t_address, register_contents);
}

/*******************************************************************************
 * @brief Set the data rate of the ADS1120.
 *
 * @param t_value - The new data rate value.
 *
 * @return Returns ESP_OK for success or a non-zero value otherwise.
 *
 * @note |   Normal Mode   | Duty-Cycle Mode |   Turbo Mode    |
 * @note |-----------------|-----------------|-----------------|
 * @note | 000 =   20 SPS  | 000 =    5 SPS  | 000 =   40 SPS  |
 * @note | 001 =   45 SPS  | 001 =   11 SPS  | 001 =   90 SPS  |
 * @note | 010 =   90 SPS  | 010 =   22 SPS  | 010 =  180 SPS  |
 * @note | 011 =  175 SPS  | 011 =   44 SPS  | 011 =  350 SPS  |
 * @note | 100 =  330 SPS  | 100 =   82 SPS  | 100 =  660 SPS  |
 * @note | 101 =  600 SPS  | 101 =  150 SPS  | 101 = 1200 SPS  |
 * @note | 110 = 1000 SPS  | 110 =  250 SPS  | 110 = 2000 SPS  |
 * @note | 111 = Reserved  | 111 = Reserved  | 111 = Reserved  |
 *******************************************************************************/
esp_err_t ADS1120::setDataRate(uint8_t t_value)
{
    if (t_value > 0x07) {
        t_value = 0x00;
    }

    t_value = t_value << 5; // Shift to match with mask
    return writeRegisterMasked(t_value, ADS1120_REG_MASK_DATARATE, ADS1120_CONFIG_REG1_ADDRESS);
}

/*******************************************************************************
 * @brief Resets the ADS1120.
 *
 * @return Returns ESP_OK for success or a non-zero value otherwise.
 *******************************************************************************/
esp_err_t ADS1120::reset() { return sendCommand(ADS1120_CMD_RESET); }

/*******************************************************************************
 * @brief Start syncing with the ADS1120.
 *
 * @return Returns ESP_OK for success or a non-zero value otherwise.
 *******************************************************************************/
esp_err_t ADS1120::startSync() { return sendCommand(ADS1120_CMD_START_SYNC); }

/*******************************************************************************
 * @brief Powers down the ADS1120.
 *
 * @return Returns ESP_OK for success or a non-zero value otherwise.
 *******************************************************************************/
esp_err_t ADS1120::powerDown() { return sendCommand(ADS1120_CMD_PWRDWN); }

esp_err_t ADS1120::rdata() { return sendCommand(ADS1120_CMD_RDATA); }