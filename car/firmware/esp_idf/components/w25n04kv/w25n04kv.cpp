#include <stdio.h>
#include <string.h>

#include <array>
#include <esp_log.h>
#include <vector>
#include <w25n04kv.hpp>

static const char* TAG = "w25n04kv";

W25N04KV::W25N04KV() {}

/*******************************************************************************
 * @brief Sends an SPI transaction to the W25N04KV device.
 *
 * @param t_init_param   - The initialization parameters for the W25N04KV.
 *******************************************************************************/
esp_err_t W25N04KV::init(w25n04kv_init_param_t t_init_param)
{
    wp_pin_ = t_init_param.wp_pin;

    esp_err_t ret;
    spi_device_interface_config_t spi_device_cfg;
    memset(&spi_device_cfg, 0, sizeof(spi_device_interface_config_t));

    spi_device_cfg.mode           = 3;
    spi_device_cfg.clock_speed_hz = 5000000; // Update based on actual clockspeed. Max 104 MHz
    spi_device_cfg.spics_io_num   = t_init_param.cs_pin;
    spi_device_cfg.flags          = SPI_DEVICE_HALFDUPLEX;
    spi_device_cfg.queue_size     = 1;
    spi_device_cfg.pre_cb         = NULL;
    spi_device_cfg.post_cb        = NULL;

    if (wp_pin_ != GPIO_NUM_NC) {
        ret = gpio_set_direction(wp_pin_, GPIO_MODE_OUTPUT);
        if (ret) {
            return ret;
        }

        ret = gpio_set_level(wp_pin_, 0);
        if (ret) {
            return ret;
        }
    }

    ret = spi_bus_add_device(t_init_param.spi_host, &spi_device_cfg, &(spi_dev_));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to SPI Bus: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Added device to SPI Bus");

    ret = reset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset device: %d", ret);
        return ret;
    }

    vTaskDelay(5);

    ret = disableWriteProtection();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable write protection on device: %d", ret);
        return ret;
    }

    vTaskDelay(5);

    ESP_LOGI(TAG, "Initialized W25N04KV Device");

    return ESP_OK;
}

/*******************************************************************************
 * @brief Executes an SPI transaction to the W25N04KV.
 *
 * @param op_code           Operation Code
 * @param rx_data           Data buffer to read from SPI Flash. The length of the read
 *                          is dictated by the length of rx_data.
 * @param address           Page address / Block address / Buffer Index
 * @param address_length    Length of address.
 * @param dummy_byte_len    The number of dummy bits to add between address and rx/tx_data
 * @param tx_data           Data buffer to send to SPI Flash. The length of the write
 *                          is dictated by the length of rx_data.
 *******************************************************************************/
esp_err_t W25N04KV::transfer(const uint8_t op_code, std::vector<uint8_t>& rx_data,
                             const uint64_t address = 0,
                             const uint8_t address_length = 0,
                             const uint8_t dummy_byte_len        = 0,
                             const std::vector<uint8_t>& tx_data = std::vector<uint8_t>(0))
{
    spi_transaction_ext_t ext_t;

    ext_t.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;

    // Set Length of Command, Address, and Dummy Bits
    ext_t.command_bits = 8;
    ext_t.address_bits = address_length;
    ext_t.dummy_bits   = dummy_byte_len;

    // Set command bits
    ext_t.base.cmd = op_code;

    // Set address bits
    ext_t.base.addr = address;

    // Set tx data buffer
    ext_t.base.tx_buffer = tx_data.data();
    ext_t.base.length    = tx_data.size() << 3;

    // Set rx data buffer
    ext_t.base.rx_buffer = rx_data.data();
    ext_t.base.rxlength  = rx_data.size() << 3;

    esp_err_t err = spi_device_polling_transmit(spi_dev_, &(ext_t.base));
    if (err) {
        return err;
    }

    return ESP_OK;
}

esp_err_t W25N04KV::reset(void)
{
    std::vector<uint8_t> dummy_rx;
    return transfer(W25N04KV_OP_CODE_RESET, dummy_rx);
}

esp_err_t W25N04KV::enableWrite(void)
{
    std::vector<uint8_t> dummy_rx;
    return transfer(W25N04KV_OP_CODE_WRITE_ENABLE, dummy_rx);
}

esp_err_t W25N04KV::disableWriteProtection(void)
{
    std::vector<uint8_t> tx_data            = {0x00};

    std::vector<uint8_t> dummy_rx;
    return transfer(W25N04KV_OP_CODE_WRITE_STAT_REG, dummy_rx, 0x0A, 8 , 0, tx_data);
};

esp_err_t W25N04KV::eraseBlock(const uint64_t block_address)
{
    esp_err_t ret = enableWrite();
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Failed to enable write: %d", ret);
        return ret;
    }

    std::vector<uint8_t> dummy_rx;

    return transfer(W25N04KV_OP_CODE_BLOCK_ERASE, dummy_rx, block_address, 24);
}

esp_err_t W25N04KV::writePage(const std::vector<uint8_t>& tx_data, uint32_t page_address)
{
    if (tx_data.size() > PAGE_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = enableWrite();
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Failed to enable write: %d", ret);
        return ret;
    }

    std::vector<uint8_t> dummy_rx;

    ret = transfer(W25N04KV_OP_CODE_DATA_LOAD, dummy_rx, 0, 16, 0, tx_data);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Failed to enable write: %d", ret);
        return ret;
    }

    return transfer(W25N04KV_OP_CODE_DATA_EXECUTE, dummy_rx, page_address, 24);
}

esp_err_t W25N04KV::readPage(std::vector<uint8_t>& rx_data, uint32_t page_address)
{
    std::vector<uint8_t> dummy_rx;

    esp_err_t ret = transfer(W25N04KV_OP_CODE_PAGE_READ_DATA, dummy_rx, page_address, 24);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Failed to read page data: %d", ret);
        return ret;
    }

    vTaskDelay(1);

    return transfer(W25N04KV_OP_CODE_READ_DATA, rx_data, 0, 16, 8);
}

/*******************************************************************************
 * @brief Reads the status of the W25N04KV device.
 * 
 * @param device_status Pointer to where the device status should be stored.
 *******************************************************************************/
esp_err_t W25N04KV::readStatus(w25n04kv_device_status_t* device_status)
{
    std::vector<uint8_t> rx_data(1);

    esp_err_t ret = transfer(W25N04KV_OP_CODE_READ_STAT_REG, rx_data, 0x0C, 8);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status register: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Value of read status register: %d", (int)rx_data[0]);

    device_status->ecc_status      = w25n04kv_ecc_status_t((rx_data[0] >> 4) & 0x03);
    device_status->program_failure = rx_data[0] & 0x08;
    device_status->erase_failure   = rx_data[0] & 0x04;
    device_status->write_enable    = rx_data[0] & 0x02;
    device_status->is_busy         = rx_data[0] & 0x01;

    return ESP_OK;
}

/*******************************************************************************
 * @brief Checks if a W25N04KV can be found given the initialization parameters.
 *******************************************************************************/
esp_err_t W25N04KV::isCorrectDevice(void)
{
    std::vector<uint8_t> rx_data(3, 0);

    esp_err_t ret = transfer(W25N04KV_OP_CODE_JEDEC_ID, rx_data, 0, 0 , 8);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read JEDEC ID: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Recorded Device ID: %d, %d, %d", int(rx_data[0]), int(rx_data[1]), int(rx_data[2]));

    if (rx_data[0] != 0xEF || rx_data[1] != 0xAA || rx_data[2] != 0x23) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}