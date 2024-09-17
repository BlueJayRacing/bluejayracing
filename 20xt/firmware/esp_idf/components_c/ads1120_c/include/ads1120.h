#pragma once
#ifndef _ADS1120_H_
#define _ADS1120_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_system.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

#define SPI_MASTER_DUMMY 0xFF
// Commands for the ADC
#define CMD_RESET 0x07
#define CMD_START_SYNC 0x08
#define CMD_PWRDWN 0x03
#define CMD_RDATA 0x1f
#define CMD_RREG 0x20
#define CMD_WREG 0x40

// Configuration registers
#define CONFIG_REG0_ADDRESS 0x00
#define CONFIG_REG1_ADDRESS 0x01
#define CONFIG_REG2_ADDRESS 0x02
#define CONFIG_REG3_ADDRESS 0x03

// Register masks for setings
// Register 0
#define REG_MASK_MUX 0xF0
#define REG_MASK_GAIN 0x0E
#define REG_MASK_PGA_BYPASS 0x01

// Register 1
#define REG_MASK_DATARATE 0xE0
#define REG_MASK_OP_MODE 0x18
#define REG_MASK_CONV_MODE 0x04
#define REG_MASK_TEMP_MODE 0x02
#define REG_MASK_BURNOUT_SOURCES 0x01

// Register 2
#define REG_MASK_VOLTAGE_REF 0xC0
#define REG_MASK_FIR_CONF 0x30
#define REG_MASK_PWR_SWITCH 0x08
#define REG_MASK_IDAC_CURRENT 0x07

// Register 3
#define REG_MASK_IDAC1_ROUTING 0xE0
#define REG_MASK_IDAC2_ROUTING 0x1C
#define REG_MASK_DRDY_MODE 0x02
#define REG_MASK_RESERVED 0x01

esp_err_t ads1120_write_reg(uint8_t address, uint8_t value);

esp_err_t ads1120_read_reg(uint8_t address, uint8_t* data_ptr);

esp_err_t ads1120_spi_init(gpio_num_t cs_pin, gpio_num_t drdy_pin);

bool ads1120_is_data_ready(void);

esp_err_t ads1120_read_adc(uint16_t* data_ptr);

esp_err_t ads1120_send_command(uint8_t command);

esp_err_t ads1120_reset(void);

esp_err_t ads1120_start_sync(void);

esp_err_t ads1120_power_down(void);

esp_err_t ads1120_rdata(void);

esp_err_t ads1120_write_reg_masked(uint8_t value, uint8_t mask, uint8_t address);

esp_err_t ads1120_set_mult(uint8_t value);

esp_err_t ads1120_set_gain(uint8_t gain);

esp_err_t ads1120_set_pga_bypass(bool value);

esp_err_t ads1120_set_data_rate(uint8_t value);

esp_err_t ads1120_set_op_mode(uint8_t value);

esp_err_t ads1120_set_conv_mode(uint8_t value);

esp_err_t ads1120_set_temp_mode(uint8_t value);

esp_err_t ads1120_set_burnout_source(bool value);

esp_err_t ads1120_set_volt_ref(uint8_t value);

esp_err_t ads1120_set_fir(uint8_t value);

esp_err_t ads1120_set_power_switch(uint8_t value);

esp_err_t ads1120_set_idac_current(uint8_t value);

esp_err_t ads1120_set_idac1_rout(uint8_t value);

esp_err_t ads1120_set_idac2_rout(uint8_t value);

esp_err_t ads1120_set_drdy_mode(uint8_t value);

#ifdef __cplusplus
}
#endif

#endif