#pragma once
#ifndef _W25N04KV_HPP_
#define _W25N04KV_HPP_

#include <array>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_system.h>
#include <vector>

#define W25N04KV_OP_CODE_JEDEC_ID       0x9F
#define W25N04KV_OP_CODE_RESET          0xFF
#define W25N04KV_OP_CODE_READ_STAT_REG  0x0F
#define W25N04KV_OP_CODE_WRITE_STAT_REG 0x1F
#define W25N04KV_OP_CODE_WRITE_ENABLE   0x06
#define W25N04KV_OP_CODE_WRITE_DISABLE  0x04
#define W25N04KV_OP_CODE_BLOCK_ERASE    0xD8
#define W25N04KV_OP_CODE_DATA_LOAD      0x02
#define W25N04KV_OP_CODE_DATA_EXECUTE   0x10
#define W25N04KV_OP_CODE_PAGE_READ_DATA 0x13
#define W25N04KV_OP_CODE_READ_DATA      0x03

typedef struct w25n04kv_init_param {
    gpio_num_t cs_pin;
    gpio_num_t wp_pin;
    spi_host_device_t spi_host;
} w25n04kv_init_param_t;

/** IN_TOL stands for in tolerance error
 *  OUT_TOL stands for out of tolerance error
 */
typedef enum w25n04kv_ecc_status
{
    NO_ERROR                = 0x0,
    IN_TOL_ERROR_CORRECTED  = 0x1,
    ERROR_NOT_CORRECTED     = 0x2,
    OUT_TOL_ERROR_CORRECTED = 0x3
} w25n04kv_ecc_status_t;

typedef struct w25n04kv_device_status {
    w25n04kv_ecc_status_t ecc_status;
    bool program_failure;
    bool erase_failure;
    bool write_enable;
    bool is_busy;
} w25n04kv_device_status_t;

class W25N04KV {
  public:
    W25N04KV();
    esp_err_t init(w25n04kv_init_param_t t_init_param);
    esp_err_t reset(void);
    esp_err_t eraseBlock(const std::array<uint8_t, 3>& address);
    esp_err_t writePage(const std::vector<uint8_t>& tx_data, uint32_t page_address);
    esp_err_t readPage(std::vector<uint8_t>& rx_data, uint32_t page_address);
    esp_err_t readStatus(w25n04kv_device_status_t* device_status);
    esp_err_t isCorrectDevice(void);

  private:
    esp_err_t enableWrite(void);
    esp_err_t disableWriteProtection(void);
    esp_err_t transfer(const uint8_t op_code, std::vector<uint8_t>& rx_data, const std::vector<uint8_t>& address,
                       const uint8_t dummy_byte_len, const std::vector<uint8_t>& tx_data);

  public:
    const static int NUM_PAGES = (1 << 17);
    const static int PAGE_SIZE = (1 << 11);

  private:
    spi_device_handle_t spi_dev_;
    gpio_num_t wp_pin_;
};

#endif