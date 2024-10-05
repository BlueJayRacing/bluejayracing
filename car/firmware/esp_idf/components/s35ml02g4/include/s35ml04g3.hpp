#pragma once
#ifndef _S35ML04G3_HPP_
#define _S35ML04G3_HPP_

#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>

class S35ML04G3 {
    public:
        S35ML04G3();
        esp_err_t init(const gpio_num_t t_cs_pin, const gpio_num_t t_wp_pin, const spi_host_device_t t_spi_host);
        esp_err_t transfer(const uint8_t op_code, const uint8_t* addr, const uint8_t addr_len, const uint8_t* data, 
                            uint16_t data_len, uint8_t* output, const uint16_t out_len, const uint8_t dummy_len);
        esp_err_t reset(void);
        esp_err_t write_enable(void);
        esp_err_t write_disable(void);
        esp_err_t block_erase(const uint8_t* addr, const uint8_t addr_len);
        esp_err_t block_erase(const uint8_t* addr, const uint8_t addr_len);
        esp_err_t program_execute(const uint8_t* addr, const uint8_t addr_len);
        esp_err_t page_read(const uint8_t* addr, const uint8_t addr_len);
        esp_err_t read_buffer(const uint8_t* addr, const uint8_t addr_len, const uint8_t* data, const uint8_t data_len, const uint8_t* output, const uint8_t output_len);


    private:
        spi_device_handle_t spi_dev_;
        gpio_num_t wp_pin_; 
};

#endif