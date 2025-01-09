#pragma once
#ifndef _ADS1120_HPP_
#define _ADS1120_HPP_

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

#define ADS1120_SPI_MASTER_DUMMY 0xFF
// Commands for the ADC
#define ADS1120_CMD_RESET      0x07
#define ADS1120_CMD_START_SYNC 0x08
#define ADS1120_CMD_PWRDWN     0x03
#define ADS1120_CMD_RDATA      0x1f
#define ADS1120_CMD_RREG       0x20
#define ADS1120_CMD_WREG       0x40

// Configuration registers
#define ADS1120_CONFIG_REG0_ADDRESS 0x00
#define ADS1120_CONFIG_REG1_ADDRESS 0x01
#define ADS1120_CONFIG_REG2_ADDRESS 0x02
#define ADS1120_CONFIG_REG3_ADDRESS 0x03

// Register masks for setings
// Register 0
#define ADS1120_REG_MASK_MUX        0xF0
#define ADS1120_REG_MASK_GAIN       0x0E
#define ADS1120_REG_MASK_PGA_BYPASS 0x01

// Register 1
#define ADS1120_REG_MASK_DATARATE        0xE0
#define ADS1120_REG_MASK_OP_MODE         0x18
#define ADS1120_REG_MASK_CONV_MODE       0x04
#define ADS1120_REG_MASK_TEMP_MODE       0x02
#define ADS1120_REG_MASK_BURNOUT_SOURCES 0x01

// Register 2
#define ADS1120_REG_MASK_VOLTAGE_REF  0xC0
#define ADS1120_REG_MASK_FIR_CONF     0x30
#define ADS1120_REG_MASK_PWR_SWITCH   0x08
#define ADS1120_REG_MASK_IDAC_CURRENT 0x07

// Register 3
#define ADS1120_REG_MASK_IDAC1_ROUTING 0xE0
#define ADS1120_REG_MASK_IDAC2_ROUTING 0x1C
#define ADS1120_REG_MASK_DRDY_MODE     0x02
#define ADS1120_REG_MASK_RESERVED      0x01

typedef struct ads1120_init_param {
  gpio_num_t cs_pin;
  gpio_num_t drdy_pin;
  spi_host_device_t spi_host;
} ads1120_init_param_t;

// refer to the private class function docs to see what values to set
typedef struct ads1120_regs {
  uint8_t analog_channels;
  uint8_t volt_refs;
  uint8_t gain;
  bool pga_bypass;
  uint8_t data_rate;
  uint8_t op_mode;
  uint8_t conv_mode;
  uint8_t temp_mode;
  uint8_t burn_sources;
  uint8_t fir;
  uint8_t power_switch;
  uint8_t idac_current;
  uint8_t idac1_routing;
  uint8_t idac2_routing;
  uint8_t drdy_mode;
} ads1120_regs_t;

class ADS1120 {
  public:
    ADS1120();
    esp_err_t init(ads1120_init_param_t t_init_param);
    esp_err_t configure(ads1120_regs_t t_new_regs);
    void getRegs(ads1120_regs_t* regs);
    bool isDataReady(void) const;
    esp_err_t readADC(uint16_t* t_data) const;

  private:
    esp_err_t readRegister(uint8_t t_address, uint8_t* t_data) const;
    esp_err_t writeRegister(uint8_t t_address, uint8_t t_value);
    esp_err_t writeRegisterMasked(uint8_t t_value, uint8_t t_mask, uint8_t t_address);
    esp_err_t sendCommand(uint8_t t_command);
    esp_err_t setAnalogChannels(uint8_t t_value);
    esp_err_t setVoltageReferences(uint8_t t_value);
    esp_err_t setGain(uint8_t t_value);
    esp_err_t setPGABypass(bool t_value);
    esp_err_t setDataRate(uint8_t t_value);
    esp_err_t setOpMode(uint8_t t_value);
    esp_err_t setConversionMode(uint8_t t_value);
    esp_err_t setTemperatureMode(uint8_t t_value);
    esp_err_t setBurnoutCurrentSources(bool t_value);
    esp_err_t setFIR(uint8_t t_value);
    esp_err_t setPowerSwitch(uint8_t t_value);
    esp_err_t setIDACCurrent(uint8_t t_value);
    esp_err_t setIDAC1Routing(uint8_t t_value);
    esp_err_t setIDAC2Routing(uint8_t t_value);
    esp_err_t setDRDYMode(uint8_t t_value);
    esp_err_t reset(void);
    esp_err_t startSync(void);
    esp_err_t powerDown(void);
    esp_err_t rdata(void);

  private:
    gpio_num_t drdy_pin_;
    spi_device_handle_t spi_dev_;
    ads1120_regs_t regs_;
};

#endif