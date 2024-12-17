/******************************************************************************
 *   @file    AD717X.cpp
 *   @brief   AD717X implementation file.
 *   	      (Sort of) Compatible devices: AD7172-2, AD7172-4, AD7173-8,
 *            AD7175-2, AD7175-8, AD7176-2, AD7177-2, AD4111, AD4112, AD4114,
 *            AD4115, AD4116 (may need some library changes)
 *T           Tested devices: AD7175-8
 *   @author  tchen (travis.yu.chen@gmail.com)
 *			  Credit to the following for the base no_os AD717X library:
 *			  acozma (andrei.cozma@analog.com)
 *            dnechita (dan.nechita@analog.com)
 *******************************************************************************/
#include <iostream>
#include <stdlib.h>

#include "ad7175_2_regs.hpp"
#include "ad7175_8_regs.hpp"
#include "ad717x.hpp"

/* Error codes */
#define INVALID_VAL -1 /* Invalid argument */
#define COMM_ERR    -2 /* Communication error on receive */
#define TIMEOUT     -3 /* A timeout has occured */

AD717X::~AD717X() {}

/*******************************************************************************
 * @brief Initializes the AD717X.
 *
 * @param t_init_param   - The structure that contains the device initial
 * 		                  parameters.
 * @param t_spi_host     - The Arduino SPI Host instance/bus that the AD717X device is on.
 * @param t_cs_pin       - The chip select pin for the AD717X device.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X::init(ad717x_init_param_t& t_init_param, SPIClass* t_spi_host, int8_t t_cs_pin)
{
    int32_t ret;
    ad717x_st_reg_t* p_reg;
    uint8_t setup_i;
    uint8_t ch_index;

    ret = initRegs(t_init_param.active_device);
    if (ret < 0) {
        return ret;
    }

    spi_host_ = t_spi_host;
    cs_pin_   = t_cs_pin;

    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);

    // Reset the device interface.
    ret = reset();
    if (ret < 0)
        return ret;

    // Initialize ADC mode register.
    ret = writeRegister(AD717X_ADCMODE_REG);
    if (ret < 0)
        return ret;

    // Initialize Interface mode register.
    ret = writeRegister(AD717X_IFMODE_REG);
    if (ret < 0)
        return ret;

    // Get CRC State
    ret = updateCRCSetting();
    if (ret < 0)
        return ret;

    // Initialize registers AD717X_GPIOCON_REG through AD717X_OFFSET0_REG
    p_reg = getReg(AD717X_GPIOCON_REG);
    if (!p_reg)
        return -1;

    while (p_reg && p_reg->addr != AD717X_OFFSET0_REG) {
        if (p_reg->addr == AD717X_ID_REG) {
            p_reg++;
            continue;
        }

        ret = writeRegister(p_reg->addr);
        if (ret < 0)
            return ret;
        p_reg++;
    }

    // Read ID register to identify the part
    ret = readRegister(AD717X_ID_REG);
    if (ret < 0)
        return ret;
    device_.active_device = t_init_param.active_device;

    for (setup_i = 0; setup_i < t_init_param.setups.size(); setup_i++) {
        // Set Polarity
        ret = setPolarity(t_init_param.setups.at(setup_i).setup.bi_polar, setup_i);
        if (ret < 0)
            return ret;

        // Select the reference source
        ret = setReferenceSource(t_init_param.setups.at(setup_i).setup.ref_source, setup_i);
        if (ret < 0)
            return ret;

        // Enable reference and input buffers
        ret = enableBuffers(t_init_param.setups.at(setup_i).setup.input_buff,
                            t_init_param.setups.at(setup_i).setup.ref_buff, setup_i);
        if (ret < 0)
            return ret;

        ret = configureDeviceODR(setup_i, t_init_param.setups.at(setup_i).filter_config.odr);
        if (ret < 0)
            return ret;

        ret = setGain(t_init_param.setups.at(setup_i).gain, setup_i);
        if (ret < 0)
            return ret;
    }

    // Set Conversion Mode
    ret = setADCMode(t_init_param.mode);
    if (ret < 0)
        return ret;

    // Connect Analog Inputs, Assign Setup, Disable all channels
    for (ch_index = 0; ch_index < t_init_param.chan_map.size(); ch_index++) {
        ret = connectAnalogInput(ch_index, t_init_param.chan_map[ch_index].inputs);
        if (ret < 0)
            return ret;

        ret = assignSetup(ch_index, t_init_param.chan_map[ch_index].setup_sel);
        if (ret < 0)
            return ret;

        ret = setChannelStatus(ch_index, t_init_param.chan_map[ch_index].channel_enable);
        if (ret < 0)
            return ret;
    }

    return 0;
}

/*******************************************************************************
 * @brief  Searches through the list of registers of the driver instance and
 *         retrieves a pointer to the register that matches the given address.
 *
 * @param t_reg_addr - The address to be used to find the register.
 *
 * @return A pointer to the register if found or 0.
 *******************************************************************************/
ad717x_st_reg* AD717X::getReg(uint8_t t_reg_addr)
{
    uint8_t i;

    if (!(&device_) || device_.regs.size() == 0)
        return nullptr;

    for (i = 0; i < device_.regs.size(); i++) {
        if (device_.regs[i].addr == t_reg_addr) {
            return &device_.regs[i];
        }
    }

    return nullptr;
}

/*******************************************************************************
 * @brief Reads the value of the specified register.
 *
 * @param t_addr - The address of the register to be read. The value will be stored
 *                 inside the register structure that holds info about this register.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X::readRegister(uint8_t t_addr)
{
    int32_t ret                     = 0;
    std::array<uint8_t, 8> send_buf = {0, 0, 0, 0, 0, 0, 0, 0};
    std::array<uint8_t, 8> ret_buf  = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t i                       = 0;
    uint8_t check8                  = 0;
    std::array<uint8_t, 8> msg_buf  = {0, 0, 0, 0, 0, 0, 0, 0};
    ad717x_st_reg_t* pReg;

    pReg = getReg(t_addr);
    if (!pReg)
        return INVALID_VAL;

    // Build the Command word
    send_buf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD | AD717X_COMM_REG_RA(pReg->addr);

    spi_host_->beginTransaction(settings_);
    digitalWrite(cs_pin_, LOW);

    spi_host_->transfer(send_buf.data(), ret_buf.data(),
                        (device_.useCRC != AD717X_DISABLE) ? pReg->size + 2 : pReg->size + 1);

    digitalWrite(cs_pin_, HIGH);
    spi_host_->endTransaction();

    if (ret < 0)
        return ret;

    // Check the CRC
    if (device_.useCRC == AD717X_USE_CRC) {
        msg_buf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD | AD717X_COMM_REG_RA(pReg->addr);
        for (i = 1; i < pReg->size + 2; ++i) {
            msg_buf[i] = ret_buf[i];
        }
        check8 = computeCRC8(msg_buf.data(), pReg->size + 2);
    }
    if (device_.useCRC == AD717X_USE_XOR) {
        msg_buf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD | AD717X_COMM_REG_RA(pReg->addr);
        for (i = 1; i < pReg->size + 2; ++i) {
            msg_buf[i] = ret_buf[i];
        }
        check8 = computeXOR8(msg_buf.data(), pReg->size + 2);
    }

    if (check8 != 0) {
        // ReadRegister checksum failed.
        return COMM_ERR;
    }

    // Build the result
    pReg->value = 0;
    for (i = 1; i < pReg->size + 1; i++) {
        pReg->value <<= 8;
        pReg->value += ret_buf[i];
    }

    return ret;
}

/*******************************************************************************
 * @brief Writes the value of the specified register.
 *
 * @param t_addr - The address of the register to be written with the value stored
 *                 inside the register structure that holds info about this
 *                 register.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X::writeRegister(uint8_t t_addr)
{
    int32_t ret                   = 0;
    int32_t reg_val               = 0;
    std::array<uint8_t, 8> wr_buf = {0, 0, 0, 0, 0, 0, 0, 0};
    std::array<uint8_t, 8> ret_buf;
    uint8_t i    = 0;
    uint8_t crc8 = 0;
    ad717x_st_reg_t* p_reg;

    p_reg = getReg(t_addr);
    if (!p_reg)
        return INVALID_VAL;

    // Build the Command word
    wr_buf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_WR | AD717X_COMM_REG_RA(p_reg->addr);

    // Fill the write buffer
    reg_val = p_reg->value;
    for (i = 0; i < p_reg->size; i++) {
        wr_buf[p_reg->size - i] = reg_val & 0xFF;
        reg_val >>= 8;
    }

    // Compute the CRC
    if (device_.useCRC != AD717X_DISABLE) {
        crc8                    = computeCRC8(wr_buf.data(), p_reg->size + 1);
        wr_buf[p_reg->size + 1] = crc8;
    }

    spi_host_->beginTransaction(settings_);
    digitalWrite(cs_pin_, LOW);

    spi_host_->transfer(wr_buf.data(), ret_buf.data(),
                        (device_.useCRC != AD717X_DISABLE) ? p_reg->size + 2 : p_reg->size + 1);

    digitalWrite(cs_pin_, HIGH);
    spi_host_->endTransaction();

    return ret;
}

/*******************************************************************************
 * @brief Resets the device.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X::reset(void)
{
    int32_t ret                    = 0;
    std::array<uint8_t, 8> rst_buf = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::array<uint8_t, 8> ret_buf;

    spi_host_->beginTransaction(settings_);
    digitalWrite(cs_pin_, LOW);

    spi_host_->transfer(rst_buf.data(), ret_buf.data(), rst_buf.size());

    digitalWrite(cs_pin_, HIGH);
    spi_host_->endTransaction();

    return ret;
}

/*******************************************************************************
 * @brief Waits until a new conversion result is available.
 *
 * @param t_timeout  - Count representing the number of polls to be done until the
 *                     function returns if no new data is available.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X::waitForReady(uint32_t t_timeout)
{
    ad717x_st_reg_t* status_reg;
    int32_t ret;
    int8_t ready = 0;

    status_reg = getReg(AD717X_STATUS_REG);
    if (!status_reg)
        return INVALID_VAL;

    while (!ready && --t_timeout) {
        // Read the value of the Status Register
        ret = readRegister(AD717X_STATUS_REG);
        if (ret < 0)
            return ret;

        // Check the RDY bit in the Status Register
        ready = (status_reg->value & AD717X_STATUS_REG_RDY) == 0;
    }

    return t_timeout ? 0 : TIMEOUT;
}

/*******************************************************************************
 * @brief Reads the conversion result from the device.
 *
 * @param t_p_data  - Pointer to store the read data.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X::readData(int32_t* t_p_data)
{
    ad717x_st_reg_t* data_reg;
    int32_t ret;

    data_reg = getReg(AD717X_DATA_REG);
    if (!data_reg)
        return INVALID_VAL;

    // Update the data register length with respect to device and options
    ret = computeDataregSize();

    // Read the value of the Status Register
    ret |= readRegister(AD717X_DATA_REG);

    // Get the read result
    *t_p_data = data_reg->value;

    return ret;
}

/******************************************************************************
 * @brief Computes data register read size to account for bit number and status
 * 		 read.
 *
 * @return 0 in case of success or negative code in case of failure.
 *******************************************************************************/
int32_t AD717X::computeDataregSize()
{
    ad717x_st_reg_t* reg_ptr;
    ad717x_st_reg_t* data_reg_ptr;
    uint16_t case_var;

    // Get interface mode register pointer
    reg_ptr = getReg(AD717X_IFMODE_REG);
    // Get data register pointer
    data_reg_ptr = getReg(AD717X_DATA_REG);
    case_var     = reg_ptr->value & (AD717X_IFMODE_REG_DATA_STAT | AD717X_IFMODE_REG_DATA_WL16);

    // Compute data register size
    data_reg_ptr->size = 3;
    if ((case_var & AD717X_IFMODE_REG_DATA_WL16) == AD717X_IFMODE_REG_DATA_WL16)
        data_reg_ptr->size--;
    if ((case_var & AD717X_IFMODE_REG_DATA_STAT) == AD717X_IFMODE_REG_DATA_STAT)
        data_reg_ptr->size++;

    // Get ID register pointer
    reg_ptr = getReg(AD717X_ID_REG);

    // If the part is 32/24 bit wide add a byte to the read
    if ((reg_ptr->value & AD717X_ID_REG_MASK) == AD7177_2_ID_REG_VALUE)
        data_reg_ptr->size++;

    return 0;
}

/******************************************************************************
 * @brief Computes the CRC checksum for a data buffer.
 *
 * @param t_p_buf    - Data buffer
 * @param t_buf_size - Data buffer size in bytes
 *
 * @return Returns the computed CRC checksum.
 *******************************************************************************/
uint8_t AD717X::computeCRC8(uint8_t* t_p_buf, uint8_t t_buf_size)
{
    uint8_t i   = 0;
    uint8_t crc = 0;

    while (t_buf_size) {
        for (i = 0x80; i != 0; i >>= 1) {
            if (((crc & 0x80) != 0) != ((*t_p_buf & i) != 0)) { // MSB of CRC register XOR input Bit from Data
                crc <<= 1;
                crc ^= AD717X_CRC8_POLYNOMIAL_REPRESENTATION;
            } else {
                crc <<= 1;
            }
        }
        t_p_buf++;
        t_buf_size--;
    }
    return crc;
}

/*******************************************************************************
 * @brief Computes the XOR checksum for a data buffer.
 *
 * @param t_p_buf    - Data buffer
 * @param t_buf_size - Data buffer size in bytes
 *
 * @return Returns the computed XOR checksum.
 *******************************************************************************/
uint8_t AD717X::computeXOR8(uint8_t* t_p_buf, uint8_t t_buf_size)
{
    uint8_t xor_value = 0;

    while (t_buf_size) {
        xor_value ^= *t_p_buf;
        t_p_buf++;
        t_buf_size--;
    }
    return xor_value;
}

/*******************************************************************************
 * @brief Updates the CRC settings.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X::updateCRCSetting()
{
    ad717x_st_reg_t* interface_reg;

    interface_reg = getReg(AD717X_IFMODE_REG);
    if (!interface_reg)
        return INVALID_VAL;

    // Get CRC State.
    if (AD717X_IFMODE_REG_CRC_STAT(interface_reg->value)) {
        device_.useCRC = AD717X_USE_CRC;
    } else if (AD717X_IFMODE_REG_XOR_STAT(interface_reg->value)) {
        device_.useCRC = AD717X_USE_XOR;
    } else {
        device_.useCRC = AD717X_DISABLE;
    }

    return 0;
}

/*******************************************************************************
 * @brief Set channel status - Enable/Disable
 *
 * @param channel_id     - Channel ID (number) of the channel whose status is to be set.
 * @param channel_status - Required status of the channel-True in case of Enable
 *			    	      and False in case of Disable
 *
 * @return Returns 0 for success or negative error code in case of failure.
 *******************************************************************************/
int32_t AD717X::setChannelStatus(uint8_t t_channel_id, bool t_channel_status)
{
    ad717x_st_reg_t* chn_reg;

    // Point to the Channel register
    chn_reg = getReg(AD717X_CHMAP0_REG + t_channel_id);
    if (!chn_reg)
        return -EINVAL;

    if (t_channel_status)
        // Assign the Channel enable bit and write to channel register
        chn_reg->value |= AD717X_CHMAP_REG_CH_EN;
    else
        chn_reg->value &= ~(AD717X_CHMAP_REG_CH_EN);

    int ret = writeRegister(AD717X_CHMAP0_REG + t_channel_id);
    if (ret < 0)
        return ret;

    return 0;
}

/*******************************************************************************
 * @brief Set ADC Mode
 *
 * @param t_adc_mode   - ADC Mode to be configured
 *
 * @return Returns 0 for success or negative error code in case of failure.
 ******************************************************************************/
int32_t AD717X::setADCMode(enum ad717x_mode t_adc_mode)
{
    ad717x_st_reg_t* adc_mode_reg;

    // Retrieve the ADC Mode register
    adc_mode_reg = getReg(AD717X_ADCMODE_REG);
    if (!adc_mode_reg)
        return -EINVAL;

    // Clear the Mode[6:4] bits in the ADC Mode Register
    adc_mode_reg->value &= ~(AD717X_ADCMODE_REG_MODE_MSK);

    // Set the required conversion mode, write to register
    adc_mode_reg->value |= AD717X_ADCMODE_REG_MODE(t_adc_mode);
    if (writeRegister(AD717X_ADCMODE_REG) < 0)
        return -EINVAL;

    device_.mode = t_adc_mode;

    return 0;
}

/*******************************************************************************
 * @brief Set Analog Inputs to channel
 *
 * @param t_channel_id      - Channel whose Analog input is to be configured
 * @param t_analog_input    - Analog Inputs to the Channel
 *
 * @return Returns 0 for success or negative error code in case of failure.
 *****************************************************************************/
int32_t AD717X::connectAnalogInput(uint8_t t_channel_id, union ad717x_analog_inputs t_analog_input)
{
    ad717x_st_reg_t* channel_reg;

    // Retrieve the channel register
    channel_reg = getReg(AD717X_CHMAP0_REG + t_channel_id);
    if (!channel_reg)
        return -EINVAL;

    switch ((uint8_t)device_.active_device) {
    case ID_AD4111:
    case ID_AD4112:
    case ID_AD4114:
    case ID_AD4115:
    case ID_AD4116:
        // Clear and Set the required analog input pair to channel
        channel_reg->value &= ~AD717x_CHANNEL_INPUT_MASK;
        channel_reg->value |= AD4111_CHMAP_REG_INPUT(t_analog_input.input_pairs);
        if (writeRegister(AD717X_CHMAP0_REG + t_channel_id) < 0)
            return -EINVAL;
        break;

    case ID_AD7172_4:
    case ID_AD7173_8:
    case ID_AD7175_2:
    case ID_AD7175_8:
    case ID_AD7176_2:
    case ID_AD7177_2:
    case ID_AD7172_2:
        // Select the Positive Analog Input
        channel_reg->value &= ~AD717X_CHMAP_REG_AINPOS_MSK;
        channel_reg->value |= AD717X_CHMAP_REG_AINPOS(t_analog_input.ainp.pos_input);

        // Select the Negative Analog Input
        channel_reg->value &= ~AD717X_CHMAP_REG_AINNEG_MSK;
        channel_reg->value |= AD717X_CHMAP_REG_AINNEG(t_analog_input.ainp.neg_input);
        if (writeRegister(AD717X_CHMAP0_REG + t_channel_id) < 0)
            return -EINVAL;
        break;

    default:
        return -EINVAL;
    }

    return 0;
}

/*******************************************************************************
 * @brief Assign Setup to Channel
 *
 * @param t_channel_id - Channel ID (number)
 * @param t_setup      - Setup ID (number)
 *
 * @return Returns 0 for success or negative error code in case of failure.
 ******************************************************************************/
int32_t AD717X::assignSetup(uint8_t t_channel_id, uint8_t t_setup)
{
    ad717x_st_reg_t* p_reg;

    // Retrieve the Channel Register
    p_reg = getReg(AD717X_CHMAP0_REG + t_channel_id);
    if (!p_reg)
        return -EINVAL;

    // Assign set up to the chosen channel
    p_reg->value &= ~AD717X_CHMAP_REG_SETUP_SEL_MSK;
    p_reg->value |= AD717X_CHMAP_REG_SETUP_SEL(t_setup);

    if (writeRegister(AD717X_CHMAP0_REG + t_channel_id) < 0)
        return -EINVAL;

    return 0;
}

/*******************************************************************************
 * @brief Set Polarity
 *
 * @param t_bipolar  - Polarity Select:True in case of Bipolar, False in case of Unipolar
 * @param t_setup_id - Setup ID (number)
 *
 * @return Returns 0 for success or negative error code in case of failure.
 *****************************************************************************/
int32_t AD717X::setPolarity(bool t_bipolar, uint8_t t_setup_id)
{
    ad717x_st_reg_t* setup_reg;

    // Retrieve the SETUPCON Register
    setup_reg = getReg(AD717X_SETUPCON0_REG + t_setup_id);
    if (!setup_reg)
        return -EINVAL;

    // Set the BI_UNIPOLAR bit in case of BIPOLAR operation
    if (t_bipolar)
        setup_reg->value |= AD717X_SETUP_CONF_REG_BI_UNIPOLAR;
    else
        setup_reg->value &= ~(AD717X_SETUP_CONF_REG_BI_UNIPOLAR);

    if (writeRegister(AD717X_SETUPCON0_REG + t_setup_id) < 0)
        return -EINVAL;

    return 0;
}

/*******************************************************************************
 * @brief Select the reference source
 *
 * @param t_ref_source - Reference source
 * @param t_setup_id - Setup ID (Number)
 *
 * @return Returns 0 for success or negative error code in case of failure.
 ******************************************************************************/
int32_t AD717X::setReferenceSource(ad717x_ref_source_t t_ref_source, uint8_t t_setup_id)
{
    ad717x_st_reg_t* setup_reg;
    ad717x_st_reg_t* adc_mode_reg;

    if (!&device_)
        return -EINVAL;

    /* Retrieve the SETUPCON Register */
    setup_reg = getReg(AD717X_SETUPCON0_REG + t_setup_id);
    if (!setup_reg)
        return -EINVAL;

    /* Choose the reference source for the selected setup */
    setup_reg->value &= ~AD717X_SETUP_CONF_REG_REF_SEL_MSK;
    setup_reg->value |= (AD717X_SETUP_CONF_REG_REF_SEL(t_ref_source));

    if (writeRegister(AD717X_SETUPCON0_REG + t_setup_id) < 0)
        return -EINVAL;

    /* Enable the REF_EN Bit in case of Internal reference */
    if (t_ref_source == INTERNAL_REF) {
        /* Retrieve the ADC Mode reigster */
        adc_mode_reg = getReg(AD717X_ADCMODE_REG);
        if (!adc_mode_reg)
            return -EINVAL;

        /* Set the REF_EN Bit */
        adc_mode_reg->value |= AD717X_ADCMODE_REG_REF_EN;
        if (writeRegister(AD717X_ADCMODE_REG) < 0)
            return -EINVAL;
        device_.ref_en = true;
    }

    return 0;
}

/*******************************************************************************
 * @brief Enable Input Buffer
 *
 * @param t_inbuf_en - Enable Input Buffer
 * @param t_refbuf_en - Enable Reference Buffer
 * @param t_setup_id - Setup ID (Number)
 *
 * @return Returns 0 for success or negative error code in case of failure.
 ******************************************************************************/
int32_t AD717X::enableBuffers(bool t_inbuf_en, bool t_refbuf_en, uint8_t t_setup_id)
{
    ad717x_st_reg* setup_reg;

    if (!&device_)
        return -EINVAL;

    /* Retrieve the SETUPCON Register */
    setup_reg = getReg(AD717X_SETUPCON0_REG + t_setup_id);
    if (!setup_reg)
        return -EINVAL;

    if (t_inbuf_en)
        /* Enable input buffer for the chosen set up */
        setup_reg->value |= (AD717X_SETUP_CONF_REG_AINBUF_P | AD717X_SETUP_CONF_REG_AINBUF_N);
    else
        setup_reg->value &= (~(AD717X_SETUP_CONF_REG_AINBUF_P | AD717X_SETUP_CONF_REG_AINBUF_N));
    if (t_refbuf_en)
        /* Enable reference buffer for the chosen set up */
        setup_reg->value |= (AD717X_SETUP_CONF_REG_REFBUF_P | AD717X_SETUP_CONF_REG_REFBUF_N);
    else
        setup_reg->value &= (~(AD717X_SETUP_CONF_REG_REFBUF_P | AD717X_SETUP_CONF_REG_REFBUF_N));

    if (writeRegister(AD717X_SETUPCON0_REG + t_setup_id) < 0)
        return -EINVAL;

    return 0;
}

/*******************************************************************************
 * @brief Configure ODR for the device
 *
 * @param t_setup_id - Associated Setup ID (Number)
 * @param t_odr_sel - ODR[4:0] bitfield value as a decimal
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD717X::configureDeviceODR(uint8_t t_setup_id, uint8_t t_odr_sel)
{
    ad717x_st_reg_t* filtcon_reg;
    int32_t ret;

    // Retrieve the FILTCON register
    filtcon_reg = getReg(AD717X_FILTCON0_REG + t_setup_id);
    if (!filtcon_reg) {
        return -EINVAL;
    }

    // Clear the ODR bits, configure the requested ODR
    filtcon_reg->value &= ~(AD717x_ODR_MSK);
    filtcon_reg->value |= AD717X_FILT_CONF_REG_ODR(t_odr_sel);

    ret = writeRegister(AD717X_FILTCON0_REG + t_setup_id);
    if (ret) {
        return ret;
    }

    return 0;
}

/*******************************************************************************
 * @brief Perform Single Conversion
 *
 * @param t_id              - Channel ID (number) requested
 * @param t_adc_raw_data    - ADC Raw Value
 *
 * @return Returns 0 for success or negative error code in case of failure.
 ******************************************************************************/
int32_t AD717X::singleRead(uint8_t t_id, int32_t* t_adc_raw_data)
{
    int ret;

    /* Enable the requested channel */
    ret = setChannelStatus(t_id, true);
    if (ret < 0)
        return ret;

    /* Set Mode to Single Conversion */
    ret = setADCMode(SINGLE);
    if (ret < 0)
        return ret;

    /* Wait for Conversion completion */
    ret = waitForReady(AD717X_CONV_TIMEOUT);
    if (ret < 0)
        return ret;

    /* Read the data register */
    ret = readData(t_adc_raw_data);
    if (ret < 0)
        return ret;

    /* Disable the current channel */
    return setChannelStatus(t_id, false);
}

/*******************************************************************************
 * @brief Initializes the registers for the device.
 *
 * @param t_dev_type        - device IC name
 *
 * @return Returns 0 for success or negative error code in case of failure.
 ******************************************************************************/
int32_t AD717X::initRegs(ad717x_device_type_t t_dev_type)
{
    switch (t_dev_type) {
    case ID_AD7175_8:
        device_.regs.resize(ad7175_8_regs.size());
        std::copy(ad7175_8_regs.begin(), ad7175_8_regs.end(), device_.regs.begin());
        break;
    case ID_AD7175_2:
        device_.regs.resize(ad7175_2_regs.size());
        std::copy(ad7175_2_regs.begin(), ad7175_2_regs.end(), device_.regs.begin());
        break;
    default:
        return -1;
    }

    return 0;
}

/*******************************************************************************
 * @brief Sets the gain on a setup.
 *
 * @param gain        - Gain for setup. Acceptable values are between 0 and 3.
 * @param t_setup_id  - The setup that the gain will be configured for.
 *
 * @return Returns 0 for success or negative error code in case of failure.
 ******************************************************************************/
int32_t AD717X::setGain(double gain, uint8_t t_setup_id)
{
    ad717x_st_reg_t* gain_reg;

    gain_reg = getReg(AD717X_GAIN0_REG + t_setup_id);
    if (!gain_reg) {
        return -EINVAL;
    }

    // Clear the ODR bits, configure the requested ODR
    gain_reg->value = (uint32_t)(gain * 0x555550);

    if (writeRegister(AD717X_GAIN0_REG + t_setup_id) < 0)
        return -EINVAL;

    return 0;
}

/*******************************************************************************
 * @brief Parses a status register.
 *
 * @param dev_status  - Struct to hold the results of the parsing.
 *
 * @return Returns 0 for success or negative error code in case of failure.
 ******************************************************************************/
void AD717X::parseStatusReg(ad717x_dev_status_t* t_dev_status)
{
    ad717x_st_reg_t* status_reg = getReg(AD717X_STATUS_REG);

    t_dev_status->data_ready     = !((status_reg->value & 0x80) >> 7);
    t_dev_status->adc_error      = ((status_reg->value & 0x40) >> 6);
    t_dev_status->crc_error      = ((status_reg->value & 0x20) >> 5);
    t_dev_status->reg_error      = ((status_reg->value & 0x10) >> 4);
    t_dev_status->active_channel = (status_reg->value & 0x0F);
}

int32_t AD717X::configureFilter(ad717x_filter_config_t t_filt_config, uint8_t t_setup_id)
{
    ad717x_st_reg_t* filter_reg;

    filter_reg = getReg(AD717X_FILTCON0_REG + t_setup_id);
    if (!filter_reg)
        return -EINVAL;

    uint32_t new_reg_value = 0;
    filter_reg->value      = new_reg_value;

    return 0;
}
