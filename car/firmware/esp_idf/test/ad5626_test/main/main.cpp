/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "ads1120.hpp"
#include "ad5626.hpp"

extern "C" void app_main(void)
{
    spi_bus_config_t spi_cfg;
    memset(&spi_cfg, 0, sizeof(spi_bus_config_t));
    spi_cfg.mosi_io_num = 18;
    spi_cfg.miso_io_num = -1;
    spi_cfg.sclk_io_num = 19;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;

    vTaskDelay(500);

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);

    AD5626 ad5626;

    gpio_num_t ldac_pin = GPIO_NUM_0;
    gpio_num_t cs_pin = GPIO_NUM_1;
    gpio_num_t clr_pin = GPIO_NUM_2;

    esp_err_t err = ad5626.init(cs_pin, ldac_pin, clr_pin, SPI2_HOST);
    if (err != ESP_OK)
    {
        printf("Failed to initialize ad5626\n");
    }

    uint16_t val = 0;

    for (int i = 0; i < 12; i++)
    {
        val = 0x0800 >> i;
        ad5626.setLevel(val);
        printf("Setting Level #%d!\n", i);
        vTaskDelay(500);
    }

    fflush(stdout);
}
