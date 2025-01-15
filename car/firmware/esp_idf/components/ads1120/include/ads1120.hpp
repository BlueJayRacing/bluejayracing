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

typedef enum ads1120_channels
{
    AIN0_AIN1 = 0x00,
    AIN0_AIN2 = 0x01,
    AIN0_AIN3 = 0x02,
    AIN1_AIN2 = 0x03,
    AIN1_AIN3 = 0x04,
    AIN2_AIN3 = 0x05,
    AIN1_AIN0 = 0x06,
    AIN3_AIN2 = 0x07,
    AIN0_AVSS = 0x08,
    AIN1_AVSS = 0x09,
    AIN2_AVSS = 0x0A,
    AIN3_AVSS = 0x0B,
    REF4_MON  = 0x0C,
    APWR4_MON = 0x0D,
    SHORTED   = 0x0E
} ads1120_channels_t;

typedef enum ads1120_volt_ref
{
    INTERNAL_2048 = 0x00,
    REFP0_REFN0   = 0x01,
    AIN_REF       = 0x02,
    ANALOG_SUPPLY = 0x03
} ads1120_volt_ref_t;

typedef enum ads1120_gain
{
    GAIN_1   = 0x00,
    GAIN_2   = 0x01,
    GAIN_4   = 0x02,
    GAIN_8   = 0x03,
    GAIN_16  = 0x04,
    GAIN_32  = 0x05,
    GAIN_64  = 0x06,
    GAIN_128 = 0x07
} ads1120_gain_t;

typedef enum ads1120_op_mode
{
    NORMAL     = 0x00,
    DUTY_CYCLE = 0x01,
    TURBO      = 0x02
} ads1120_op_mode_t;

typedef enum ads1120_conv_mode
{
    SINGLE_SHOT = 0x0,
    CONTINUOUS  = 0x01
} ads1120_conv_mode_t;

typedef enum ads1120_temp_mode
{
    TEMPMODE_DISABLED = 0x00,
    TEMPMODE_ENABLED  = 0x01
} ads1120_temp_mode_t;

typedef enum ads1120_fir
{
    NO_FILTER   = 0x00,
    REJ_50_60HZ = 0x01,
    REJ_50HZ    = 0x02,
    REJ_60HZ    = 0x03
} ads1120_fir_t;

typedef enum ads1120_idac_current
{
    IDAC_OFF       = 0x00,
    IDAC_ON_10uA   = 0x01,
    IDAC_ON_50uA   = 0x02,
    IDAC_ON_100uA  = 0x03,
    IDAC_ON_250uA  = 0x04,
    IDAC_ON_500uA  = 0x05,
    IDAC_ON_1000uA = 0x06,
    IDAC_ON_1500uA = 0x07
} ads1120_idac_current_t;

typedef enum ads1120_idac_routing_t
{
    ROUTING_DISABLED   = 0x00,
    AIN0_REFP1 = 0x01,
    AIN1       = 0x02,
    AIN2       = 0x03,
    AIN3_REFN1 = 0x04,
    REFP0      = 0x05,
    REFN0      = 0x06,
} ads1120_idac_routing_t;

typedef enum ads1120_drdy_mode
{
    DRDY      = 0x00,
    DRDY_DOUT = 0x01
} ads1120_drdy_mode_t;

// memset to 0 to achieve default behavior
typedef struct ads1120_regs {
    uint8_t data_rate;
    bool pga_bypass;
    bool burn_sources;
    bool power_switch;
    ads1120_fir_t fir;
    ads1120_gain_t gain;
    ads1120_op_mode_t op_mode;
    ads1120_volt_ref_t volt_refs;
    ads1120_conv_mode_t conv_mode;
    ads1120_temp_mode_t temp_mode;
    ads1120_drdy_mode_t drdy_mode;
    ads1120_channels_t channels;
    ads1120_idac_current_t idac_current;
    ads1120_idac_routing_t idac1_routing;
    ads1120_idac_routing_t idac2_routing;
} ads1120_regs_t;

class ADS1120 {
  public:
    ADS1120();
    esp_err_t init(ads1120_init_param_t t_init_param);
    esp_err_t configure(ads1120_regs_t t_new_regs);
    void getRegs(ads1120_regs_t* regs);
    bool isDataReady(void) const;
    esp_err_t readADC(int16_t* t_data) const;
    esp_err_t reset(void);
    esp_err_t startSync(void);
    esp_err_t powerDown(void);
    esp_err_t rdata(void);

  private:
    esp_err_t readRegister(uint8_t t_address, uint8_t* t_data) const;
    esp_err_t writeRegister(uint8_t t_address, uint8_t t_value);
    esp_err_t writeRegisterMasked(uint8_t t_value, uint8_t t_mask, uint8_t t_address);
    esp_err_t sendCommand(uint8_t t_command);
    esp_err_t setDataRate(uint8_t t_value);

  private:
    gpio_num_t drdy_pin_;
    spi_device_handle_t spi_dev_;
    ads1120_regs_t regs_;
};

#endif