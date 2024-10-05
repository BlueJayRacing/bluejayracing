#include <stdio.h>
#include <string.h>

#include "s35ml04g3.hpp"

S35ML04G3::S35ML04G3()
{
}

esp_err_t S35ML04G3::init(const gpio_num_t t_cs_pin, const gpio_num_t t_wp_pin, const spi_host_device_t t_spi_host)
{
    wp_pin_ = t_wp_pin;

    spi_device_interface_config_t spi_device_cfg;
    memset(&spi_device_cfg, 0, sizeof(spi_device_interface_config_t));

    spi_device_cfg.mode = 3;
    spi_device_cfg.clock_speed_hz = 1000000; //Update based on actual clockspeed. Max 104 MHz
    spi_device_cfg.spics_io_num = t_cs_pin;
    spi_device_cfg.flags = SPI_DEVICE_HALFDUPLEX;
    spi_device_cfg.queue_size = 1;
    spi_device_cfg.pre_cb = NULL;
    spi_device_cfg.post_cb = NULL;

    esp_err_t ret = gpio_set_direction(wp_pin_, GPIO_MODE_OUTPUT);
    if (ret)
    {
        return ret;
    }

    ret = gpio_set_level(wp_pin_, 0);
    if (ret)
    {
        return ret;
    }

    return spi_bus_add_device(t_spi_host, &spi_device_cfg, &(spi_dev_));
}


esp_err_t S35ML04G3::transfer(const uint8_t op_code, const uint8_t* addr, const uint8_t addr_len, const uint8_t* data, 
                            uint16_t data_len, uint8_t* output, const uint16_t out_len, const uint8_t dummy_len)
{

    spi_transaction_ext_t ext_t;

    ext_t.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
    ext_t.base.cmd = op_code;
    memcpy(&ext_t.base.addr, addr, addr_len);
    ext_t.base.length = data_len;
    ext_t.base.rxlength = out_len; 
    ext_t.base.rx_buffer = output; 
    ext_t.base.tx_buffer = data; 

    ext_t.command_bits = 8;
    ext_t.address_bits = addr_len << 3;
    ext_t.dummy_bits = dummy_len;

    esp_err_t err = spi_device_polling_transmit(spi_dev_ , &(ext_t.base));
    if (err)
    {
        return err;
    }
    return 0;
}

esp_err_t S35ML04G3::reset(void) 
{
    return transfer(0xFF, NULL, 0, NULL, 0, NULL, 0, 0);
}

esp_err_t S35ML04G3::write_enable(void) 
{
    return transfer(0x06, NULL, 0, NULL, 0, NULL, 0, 0);
}

esp_err_t S35ML04G3::write_disable(void) 
{
    return transfer(0x04, NULL, 0, NULL, 0, NULL, 0, 0);
}

esp_err_t S35ML04G3::block_erase(const uint8_t* addr, const uint8_t addr_len)
{
    return transfer(0xD8, addr, addr_len, NULL, 0, NULL, 0, 0);
}

esp_err_t S35ML04G3::block_erase(const uint8_t* addr, const uint8_t addr_len)
{
    return transfer(0xD8, addr, addr_len, NULL, 0, NULL, 0, 0);
}

esp_err_t S35ML04G3::program_execute(const uint8_t* addr, const uint8_t addr_len)
{
    return transfer(0x10, addr, addr_len, NULL, 0, NULL, 0, 0);
}

esp_err_t S35ML04G3::page_read(const uint8_t* addr, const uint8_t addr_len)
{
    return transfer(0x13, addr, addr_len, NULL, 0, NULL, 0, 0);
}

esp_err_t S35ML04G3::read_buffer(const uint8_t* addr, const uint8_t addr_len, const uint8_t* data, const uint8_t data_len, const uint8_t* output, const uint8_t output_len)
{
    return transfer(0x03, addr, addr_len, data, data_len, output, output_len, 8);
}



