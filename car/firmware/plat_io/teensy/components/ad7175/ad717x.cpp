/***************************************************************************//**
*   @file    AD717X.c
*   @brief   AD717X implementation file.
*   	     Devices: AD7172-2, AD7172-4, AD7173-8, AD7175-2, AD7175-8, AD7176-2
*            AD7177-2, AD4111, AD4112, AD4114, AD4115, AD4116
*   @author  acozma (andrei.cozma@analog.com)
*            dnechita (dan.nechita@analog.com)
*
********************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdlib.h>
#include "ad717x.hpp"

/* Error codes */
#define INVALID_VAL -1 /* Invalid argument */
#define COMM_ERR    -2 /* Communication error on receive */
#define TIMEOUT     -3 /* A timeout has occured */

int32_t AD717X::setChannelStatus(uint8_t t_channel_id, bool t_channel_status)
{
    ad717x_st_reg *chn_register;

    // Point to the Channel register
    chn_register = getReg(AD717X_CHMAP0_REG + t_channel_id);
    if (!chn_register)
        return -EINVAL;

    if (t_channel_status)
        // Assign the Channel enable bit and write to channel register
        chn_register->value |= AD717X_CHMAP_REG_CH_EN;
    else
        chn_register->value &= ~(AD717X_CHMAP_REG_CH_EN);

    int ret = writeRegister(AD717X_CHMAP0_REG + t_channel_id);
    if (ret < 0)
        return ret;
    device_.chan_map[t_channel_id].channel_enable = t_channel_status;

    return 0;
}

int32_t AD717X::setADCMode(enum ad717x_mode t_mode)
{
    ad717x_st_reg *adc_mode_reg;

    // Retrieve the ADC Mode register
    adc_mode_reg = getReg(AD717X_ADCMODE_REG);
    if (!adc_mode_reg)
        return -EINVAL;

    // Clear the Mode[6:4] bits in the ADC Mode Register
    adc_mode_reg->value &= ~(AD717X_ADCMODE_REG_MODE_MSK);

    // Set the required conversion mode, write to register
    adc_mode_reg->value |= AD717X_ADCMODE_REG_MODE(t_mode);
    if (writeRegister(AD717X_ADCMODE_REG) < 0)
        return -EINVAL;
    device_.mode = t_mode;

    return 0;
}

int32_t AD717X::connectAnalogInput(uint8_t t_channel_id, union ad717x_analog_inputs t_analog_input)
{
    ad717x_st_reg *channel_reg;

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
        channel_reg->value  &= ~AD717x_CHANNEL_INPUT_MASK;
        channel_reg->value |= AD4111_CHMAP_REG_INPUT(t_analog_input.analog_input_pairs);
        if (writeRegister(AD717X_CHMAP0_REG + t_channel_id) < 0)
            return -EINVAL;

        device_.chan_map[t_channel_id].analog_inputs.analog_input_pairs =
            t_analog_input.analog_input_pairs;
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
        channel_reg->value |=  AD717X_CHMAP_REG_AINPOS(
                               t_analog_input.ainp.pos_analog_input);

        // Select the Negative Analog Input
        channel_reg->value &= ~AD717X_CHMAP_REG_AINNEG_MSK;
        channel_reg->value |= AD717X_CHMAP_REG_AINNEG(
                              t_analog_input.ainp.neg_analog_input);
        if (writeRegister(AD717X_CHMAP0_REG + t_channel_id) < 0)
            return -EINVAL;

        device_.chan_map[t_channel_id].analog_inputs.ainp.pos_analog_input =
            t_analog_input.ainp.pos_analog_input;
        device_.chan_map[t_channel_id].analog_inputs.ainp.neg_analog_input =
            t_analog_input.ainp.neg_analog_input;
        break;

    default:
        return -EINVAL;
    }

    return 0;
}

int32_t AD717X::assignSetup(uint8_t t_channel_id, uint8_t t_setup)
{
    ad717x_st_reg *p_register;

    // Retrieve the Channel Register
    p_register = getReg(AD717X_CHMAP0_REG + t_channel_id);
    if (!p_register)
        return -EINVAL;

    // Assign set up to the chosen channel
    p_register->value &= ~AD717X_CHMAP_REG_SETUP_SEL_MSK;
    p_register->value |= AD717X_CHMAP_REG_SETUP_SEL(t_setup);

    if (writeRegister(AD717X_CHMAP0_REG + t_channel_id) < 0)
        return -EINVAL;
    device_.chan_map[t_channel_id].setup_sel = t_setup;

    return 0;
}

int32_t AD717X::setPolarity(bool t_bipolar, uint8_t t_setup_id)
{
    ad717x_st_reg* setup_reg;

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
    device_.setups[t_setup_id].bi_unipolar = t_bipolar;

    return 0;
}

int32_t AD717X::readRegister(uint8_t t_addr)
{
    int32_t ret       = 0;
    uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t i         = 0;
    uint8_t check8    = 0;
    uint8_t msgBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    ad717x_st_reg *pReg;

    pReg = getReg(t_addr);
    if (!pReg)
        return INVALID_VAL;

    // Build the Command word
    buffer[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD |
                AD717X_COMM_REG_RA(pReg->addr);

    // // Read data from the device
    // ret = no_os_spi_write_and_read(device_.spi_desc,
    //                                buffer,
    //                                ((device_.useCRC != AD717X_DISABLE) ? pReg->size + 1
    //                                 : pReg->size) + 1);
    if(ret < 0)
        return ret;

    // Check the CRC
    if(device_.useCRC == AD717X_USE_CRC) {
        msgBuf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD |
                    AD717X_COMM_REG_RA(pReg->addr);
        for(i = 1; i < pReg->size + 2; ++i) {
            msgBuf[i] = buffer[i];
        }
        check8 = computeCRC8(msgBuf, pReg->size + 2);
    }
    if(device_.useCRC == AD717X_USE_XOR) {
        msgBuf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD |
                    AD717X_COMM_REG_RA(pReg->addr);
        for(i = 1; i < pReg->size + 2; ++i) {
            msgBuf[i] = buffer[i];
        }
        check8 = computeXOR8(msgBuf, pReg->size + 2);
    }

    if(check8 != 0) {
        // ReadRegister checksum failed.
        return COMM_ERR;
    }

    // Build the result
    pReg->value = 0;
    for(i = 1; i < pReg->size + 1; i++) {
        pReg->value <<= 8;
        pReg->value += buffer[i];
    }

    return ret;
}

int32_t AD717X::writeRegister(uint8_t t_addr)
{
    int32_t ret      = 0;
    int32_t regValue = 0;
    uint8_t wrBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t i        = 0;
    uint8_t crc8     = 0;
    ad717x_st_reg *preg;

    preg = getReg(t_addr);
    if (!preg)
        return INVALID_VAL;

    // Build the Command word
    wrBuf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_WR |
               AD717X_COMM_REG_RA(preg->addr);

    // Fill the write buffer
    regValue = preg->value;
    for(i = 0; i < preg->size; i++) {
        wrBuf[preg->size - i] = regValue & 0xFF;
        regValue >>= 8;
    }

    // Compute the CRC
    if(device_.useCRC != AD717X_DISABLE) {
        crc8 = computeCRC8(wrBuf, preg->size + 1);
        wrBuf[preg->size + 1] = crc8;
    }

    // // Write data to the device
    // ret = no_os_spi_write_and_read(device_.spi_desc,
    //                                wrBuf,
    //                                (device_.useCRC != AD717X_DISABLE) ?
    //                                preg->size + 2 : preg->size + 1);

    return ret;
}

int32_t AD717X::reset()
{
    int32_t ret = 0;
    uint8_t wrBuf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    // ret = no_os_spi_write_and_read(device_.spi_desc,
    //                                wrBuf,
    //                                8);

    return ret;
}

int32_t AD717X::waitForReady(uint32_t t_timeout)
{
    ad717x_st_reg *statusReg;
    int32_t ret;
    int8_t ready = 0;

    statusReg = getReg(AD717X_STATUS_REG);
    if (!statusReg)
        return INVALID_VAL;

    while(!ready && --t_timeout) {
        // Read the value of the Status Register
        ret = readRegister(AD717X_STATUS_REG);
        if(ret < 0)
            return ret;

        // Check the RDY bit in the Status Register
        ready = (statusReg->value & AD717X_STATUS_REG_RDY) == 0;
    }

    return t_timeout ? 0 : TIMEOUT;
}

int32_t AD717X::readData(int32_t* t_p_data)
{
    ad717x_st_reg *dataReg;
    int32_t ret;

    dataReg = getReg(AD717X_DATA_REG);
    if (!dataReg)
        return INVALID_VAL;

    // Update the data register length with respect to device and options
    ret = computeDataregSize();

    // Read the value of the Status Register
    ret |= readRegister(AD717X_DATA_REG);

    // Get the read result
    *t_p_data = dataReg->value;

    return ret;
}

int32_t AD717X::computeDataregSize()
{
    ad717x_st_reg *reg_ptr;
    ad717x_st_reg *datareg_ptr;
    uint16_t case_var;

    // Get interface mode register pointer
    reg_ptr = getReg(AD717X_IFMODE_REG);
    // Get data register pointer
    datareg_ptr = getReg(AD717X_DATA_REG);
    case_var = reg_ptr->value & (AD717X_IFMODE_REG_DATA_STAT |
                                 AD717X_IFMODE_REG_DATA_WL16);

    // Compute data register size
    datareg_ptr->size = 3;
    if ((case_var & AD717X_IFMODE_REG_DATA_WL16) == AD717X_IFMODE_REG_DATA_WL16)
        datareg_ptr->size--;
    if ((case_var & AD717X_IFMODE_REG_DATA_STAT) == AD717X_IFMODE_REG_DATA_STAT)
        datareg_ptr->size++;

    // Get ID register pointer
    reg_ptr = getReg(AD717X_ID_REG);

    // If the part is 32/24 bit wide add a byte to the read
    if((reg_ptr->value & AD717X_ID_REG_MASK) == AD7177_2_ID_REG_VALUE)
        datareg_ptr->size++;

    return 0;
}

uint8_t AD717X::computeCRC8(uint8_t * t_p_buf, uint8_t t_buf_size)
{
    uint8_t i   = 0;
    uint8_t crc = 0;

    while(t_buf_size) {
        for(i = 0x80; i != 0; i >>= 1) {
            if(((crc & 0x80) != 0) != ((*t_p_buf & i) !=
                                       0)) { // MSB of CRC register XOR input Bit from Data
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

uint8_t AD717X::computeXOR8(uint8_t * t_p_buf, uint8_t t_buf_size)
{
    uint8_t xor_value = 0;

    while(t_buf_size) {
        xor_value ^= *t_p_buf;
        t_p_buf++;
        t_buf_size--;
    }
    return xor_value;
}

int32_t AD717X::updateCRCSetting()
{
    ad717x_st_reg *interfaceReg;

    interfaceReg = getReg(AD717X_IFMODE_REG);
    if (!interfaceReg)
        return INVALID_VAL;

    // Get CRC State.
    if(AD717X_IFMODE_REG_CRC_STAT(interfaceReg->value)) {
        device_.useCRC = AD717X_USE_CRC;
    } else if(AD717X_IFMODE_REG_XOR_STAT(interfaceReg->value)) {
        device_.useCRC = AD717X_USE_XOR;
    } else {
        device_.useCRC = AD717X_DISABLE;
    }

    return 0;
}

int32_t AD717X::configureDeviceODR(uint8_t t_filtcon_id, uint8_t t_odr_sel)
{
    ad717x_st_reg *filtcon_reg;
    int32_t ret;

    // Retrieve the FILTCON register
    filtcon_reg = getReg(AD717X_FILTCON0_REG + t_filtcon_id);
    if (!filtcon_reg) {
        return -EINVAL;
    }

    // Clear the ODR bits, configure the requested ODR
    filtcon_reg->value &= ~(AD717x_ODR_MSK);
    filtcon_reg->value |= AD717X_FILT_CONF_REG_ODR(t_odr_sel);

    ret = writeRegister(AD717X_FILTCON0_REG + t_filtcon_id);
    if (ret) {
        return ret;
    }

    return 0;
}

int32_t AD717X::init(ad717x_init_param t_init_param, SPIClass* t_spi_host)
{
    int32_t ret;
    ad717x_st_reg *preg;
    uint8_t setup_index;
    uint8_t ch_index;

    device_.regs = t_init_param.regs;
    device_.num_regs = t_init_param.num_regs;
	spi_host_ = t_spi_host;

    // Reset the device interface.
    ret = reset();
    if (ret < 0)
        return ret;

    // Initialize ADC mode register.
    ret = writeRegister(AD717X_ADCMODE_REG);
    if(ret < 0)
        return ret;

    // Initialize Interface mode register.
    ret = writeRegister(AD717X_IFMODE_REG);
    if(ret < 0)
        return ret;

    // Get CRC State
    ret = updateCRCSetting();
    if(ret < 0)
        return ret;

    // Initialize registers AD717X_GPIOCON_REG through AD717X_OFFSET0_REG
    preg = getReg(AD717X_GPIOCON_REG);
    if (!preg)
        return -1;

    while (preg && preg->addr != AD717X_OFFSET0_REG) {
        if (preg->addr == AD717X_ID_REG) {
            preg++;
            continue;
        }

        ret = writeRegister(preg->addr);
        if (ret < 0)
            return ret;
        preg++;
    }

    // Read ID register to identify the part
    ret = readRegister(AD717X_ID_REG);
    if(ret < 0)
        return ret;
    device_.active_device = t_init_param.active_device;
    device_.num_channels = t_init_param.num_channels;

    for (setup_index = 0; setup_index < t_init_param.num_setups; setup_index++) {
        // Set Polarity
        ret = setPolarity(t_init_param.setups[setup_index].bi_unipolar, setup_index);
        if (ret < 0)
        	return ret;

        // Select the reference source
        ret = setReferenceSource(t_init_param.setups[setup_index].ref_source, setup_index);
        if (ret < 0)
			return ret;

        // Enable reference and input buffers
        ret = enableBuffers(t_init_param.setups[setup_index].input_buff,
                            t_init_param.setups[setup_index].ref_buff,
                            setup_index);
        if (ret < 0)
			return ret;

        ret = configureDeviceODR(setup_index,
                                 t_init_param.filter_configuration[setup_index].odr);
        if (ret < 0)
			return ret;
    }

    // Set Conversion Mode
    ret = setADCMode(t_init_param.mode);
    if (ret < 0)
			return ret;

    // Connect Analog Inputs, Assign Setup, Disable all channels
    for (ch_index = 0; ch_index < t_init_param.num_channels; ch_index++) {
        ret = connectAnalogInput(ch_index,
                                 t_init_param.chan_map[ch_index].analog_inputs);
        if (ret < 0)
			return ret;

        ret = assignSetup(ch_index,
                          t_init_param.chan_map[ch_index].setup_sel);
        if (ret < 0)
			return ret;

        ret = setChannelStatus(ch_index,
                               t_init_param.chan_map[ch_index].channel_enable);
        if (ret < 0)
			return ret;
    }

	return 0;
}

AD717X::~AD717X()
{
}