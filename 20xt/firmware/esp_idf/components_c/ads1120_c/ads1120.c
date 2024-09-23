#include <stdio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <string.h>

#include "ads1120.h"

gpio_num_t ADS1120_CS_PIN;
gpio_num_t ADS1120_DRDY_PIN;
spi_device_handle_t spi_dev;

#define ADS_SPI_BUS SPI2_HOST
#define ADS_SPI_LOCK_TIMEOUT 10
#define ADS_SPI_CLOCK_SPEED_HZ 1 * 1000 * 1000 // Clock out at 1 MHz
#define ADS_SPI_MODE 1
#define ADS_CS_EN_PRE_WAIT_CYCLES 2
#define ADS_CS_EN_POST_WAIT_CYCLES 0
#define ADS_SPI_INPUT_DELAY_NS 0

esp_err_t ads1120_send_command(uint8_t command)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = command;
    t.length = 1 * 8;

    ret = spi_device_polling_transmit(spi_dev, &t); // Transmit!
    return ret;
}

esp_err_t ads1120_write_reg(uint8_t address, uint8_t value)
{

    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = (CMD_WREG | (address << 2));
    t.tx_data[1] = value;
    t.length = 2 * 8; // 2 bytes

    ret = spi_device_polling_transmit(spi_dev, &t); // Transmit!
    return ret;
}

esp_err_t ads1120_read_reg(uint8_t address, uint8_t *data_ptr)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 2 * 8;   // 2 bytes
    t.rxlength = 2 * 8; // 2 bytes
    t.tx_data[0] = (CMD_RREG | (address << 2));
    t.tx_data[1] = SPI_MASTER_DUMMY;

    ret = spi_device_polling_transmit(spi_dev, &t); // Transmit!
    *data_ptr = t.rx_data[1];
    return ret;
}

esp_err_t ads1120_spi_init(gpio_num_t cs_pin, gpio_num_t drdy_pin)
{
    // Set pins up
    ADS1120_CS_PIN = cs_pin;
    ADS1120_DRDY_PIN = drdy_pin;

    gpio_set_direction((gpio_num_t)ADS1120_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)ADS1120_DRDY_PIN, GPIO_MODE_INPUT);

    spi_device_interface_config_t devcfg = {
        .mode = ADS_SPI_MODE,
        .cs_ena_pretrans = ADS_CS_EN_PRE_WAIT_CYCLES,
        .cs_ena_posttrans = ADS_CS_EN_POST_WAIT_CYCLES,
        .clock_speed_hz = ADS_SPI_CLOCK_SPEED_HZ,
        .input_delay_ns = ADS_SPI_INPUT_DELAY_NS,
        .spics_io_num = ADS1120_CS_PIN, // CS pin
        .flags = 0,
        .queue_size = 1,
    };

    spi_bus_add_device(ADS_SPI_BUS, &devcfg, &spi_dev);
    vTaskDelay(5);

    ads1120_reset();
    vTaskDelay(5);
    return ads1120_start_sync(); // Send start/sync for continuous conversion mode
}

bool ads1120_is_data_ready()
{
    return !gpio_get_level(ADS1120_DRDY_PIN);
}

esp_err_t ads1120_read_adc(uint16_t *data_ptr)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    memset(t.tx_data, SPI_MASTER_DUMMY, 2);

    t.length = 2 * 8;   // 2 bytes
    t.rxlength = 2 * 8; // 2 bytes

    ret = spi_device_polling_transmit(spi_dev, &t); // Transmit!
    *data_ptr = t.rx_data[0];
    *data_ptr = (*data_ptr << 8) | t.rx_data[1];
    return ret;
}

esp_err_t ads1120_write_reg_masked(uint8_t value, uint8_t mask, uint8_t address)
{
    // Escribe un valor en el registro, aplicando la mascara para tocar unicamente los bits necesarios.
    // No realiza el corrimiento de bits (shift), hay que pasarle ya el valor corrido a la posicion correcta

    // Leo el contenido actual del registro
    uint8_t register_contents;
    ads1120_read_reg(address, &register_contents);

    // Cambio bit aa bit la mascara (queda 1 en los bits que no hay que tocar y 0 en los bits a modificar)
    // Se realiza un AND co el contenido actual del registro.  Quedan "0" en la parte a modificar
    register_contents = register_contents & ~mask;

    // se realiza un OR con el valor a cargar en el registro.  Ojo, valor debe estar en el posicion (shitf) correcta
    register_contents = register_contents | value;

    // Escribo nuevamente el registro
    return ads1120_write_reg(address, register_contents);
}

esp_err_t ads1120_set_mult(uint8_t value)
{
    /* Set multiplexer

    | Value | AINp | AINn |
    | ----- | ---- | ---- |
    | 0x00  | AIN0 | AIN1 |
    | 0X01  | AIN0 | AIN2 |
    | 0X02  | AIN0 | AIN3 |
    | 0X03  | AIN1 | AIN2 |
    | 0X04  | AIN1 | AIN3 |
    | 0X05  | AIN2 | AIN3 |
    | 0X06  | AIN1 | AIN0 |
    | 0X07  | AIN3 | AIN2 |
    | 0X08  | AIN0 | AVSS |
    | 0X09  | AIN1 | AVSS |
    | 0X0A  | AIN2 | AVSS |
    | 0X0B  | AIN3 | AVSS |
    | 0X0C  |  REF/4 MON  |
    | 0X0D  | APWR/4 MON  |
    | 0X0E  |   SHORTED   |
    */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x0E)
    {
        value = 0x00;
    }
    value = value << 4; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_MUX, CONFIG_REG0_ADDRESS);
}

esp_err_t ads1120_set_gain(uint8_t gain)
{
    /* Sets ADC gain. Possible values are 1, 2, 4, 8, 16, 32, 64, 128. */
    uint8_t value = 0x00;
    switch (gain)
    {
    case 1:
        value = 0x00;
        break;
    case 2:
        value = 0x01;
        break;
    case 4:
        value = 0x02;
        break;
    case 8:
        value = 0x03;
        break;
    case 16:
        value = 0x04;
        break;
    case 32:
        value = 0x05;
        break;
    case 64:
        value = 0x06;
        break;
    case 128:
        value = 0x07;
        break;
    default:
        value = 0x00;
        break;
    }
    value = value << 1; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_GAIN, CONFIG_REG0_ADDRESS);
}

esp_err_t ads1120_set_pga_bypass(bool value)
{
    /* Bypasses the PGA if true.
       PGA can only be disabled for gains 1, 2, 4.
    */
    return ads1120_write_reg_masked(value, REG_MASK_PGA_BYPASS, CONFIG_REG0_ADDRESS);
}

esp_err_t ads1120_set_data_rate(uint8_t value)
{
    /* Sets the data rate for the ADC. See table 18 in datasheet for datarates
       in various operating modes. */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x07)
    {
        value = 0x00;
    }
    value = value << 5; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_DATARATE, CONFIG_REG1_ADDRESS);
}

esp_err_t ads1120_set_op_mode(uint8_t value)
{
    /* Sets the ADC operating mode:
       0 - Normal mode
       1 - Duty-cycle mode
       2 - Turbo mode
    */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x02)
    {
        value = 0x00;
    }
    value = value << 3; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_OP_MODE, CONFIG_REG1_ADDRESS);
}

esp_err_t ads1120_set_conv_mode(uint8_t value)
{
    /* Sets the ADC conversion mode.
       0 - Single shot mode
       1 - continuous conversion mode
    */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x01)
    {
        value = 0x00;
    }
    value = value << 2; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_CONV_MODE, CONFIG_REG1_ADDRESS);
}

esp_err_t ads1120_set_temp_mode(uint8_t value)
{
    /* Controls the state of the internal temperature sensor.
       0 - Disables temperature sensor
       1 - Enables temperature sensor
    */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x01)
    {
        value = 0x00;
    }
    value = value << 1; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_TEMP_MODE, CONFIG_REG1_ADDRESS);
}

esp_err_t ads1120_set_burnout_source(bool value)
{
    /* Turns the 10uA burn-out current sources on or off. */
    return ads1120_write_reg_masked(value, REG_MASK_BURNOUT_SOURCES, CONFIG_REG1_ADDRESS);
}

esp_err_t ads1120_set_volt_ref(uint8_t value)
{
    /* Sets the voltage reference used by the ADC.
       0 - Internal 2.048 V
       1 - External on REFP0 and REFN0 inputs
       2 - External on AIN0/REFP1 and AIN3/REFN1 inputs
       3 - Use analog supply as reference
    */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x03)
    {
        value = 0x00;
    }
    value = value << 6; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_VOLTAGE_REF, CONFIG_REG2_ADDRESS);
}

esp_err_t ads1120_set_fir(uint8_t value)
{
    /* Controls the FIR filter on the ADC.
       0 - No 50 or 60 Hz rejection
       1 - Both 50 and 60 Hz rejection
       2 - 50 Hz rejection
       3 - 60 Hz rejection
    */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x03)
    {
        value = 0x00;
    }
    value = value << 4; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_FIR_CONF, CONFIG_REG2_ADDRESS);
}

esp_err_t ads1120_set_power_switch(uint8_t value)
{
    /* Configures behavior of low-side switch between AIN3/REFN1 and AVSS.
       0 - Always open
       1 - Automatically closes when START/SYNC command is sent and opens when
           ads1120_power_down command is issues.
    */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x01)
    {
        value = 0x00;
    }
    value = value << 3; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_PWR_SWITCH, CONFIG_REG2_ADDRESS);
}

esp_err_t ads1120_set_idac_current(uint8_t value)
{
    /* Set current for both IDAC1 and IDAC2 excitation sources.
       0 - Off
       1 - 10 uA
       2 - 50 uA
       3 - 100 uA
       4 - 250 uA
       5 - 500 uA
       6 - 1000 uA
       7 - 1500 uA
    */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x07)
    {
        value = 0x00;
    }
    return ads1120_write_reg_masked(value, REG_MASK_IDAC_CURRENT, CONFIG_REG2_ADDRESS);
}

esp_err_t ads1120_set_idac1_rout(uint8_t value)
{
    /* Selects where IDAC1 is routed to.
       0 - Disabled
       1 - AIN0/REFP1
       2 - AIN1
       3 - AIN2
       4 - AIN3/REFN1
       5 - REFP0
       6 - REFN0
    */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x06)
    {
        value = 0x00;
    }
    value = value << 5; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_IDAC1_ROUTING, CONFIG_REG3_ADDRESS);
}

esp_err_t ads1120_set_idac2_rout(uint8_t value)
{
    /* Selects where IDAC2 is routed to.
       0 - Disabled
       1 - AIN0/REFP1
       2 - AIN1
       3 - AIN2
       4 - AIN3/REFN1
       5 - REFP0
       6 - REFN0
    */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x06)
    {
        value = 0x00;
    }
    value = value << 2; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_IDAC2_ROUTING, CONFIG_REG3_ADDRESS);
}

esp_err_t ads1120_set_drdy_mode(uint8_t value)
{
    /* Controls the behavior of the DOUT/DRDY pin when new data are ready.
       0 - Only the dedicated DRDY pin is used  (Default)
       1 - Data ready indicated on DOUT/DRDY and DRDY
   */
    // Make sure the value is in the valid range. Otherwise set to 0x00
    if (value > 0x01)
    {
        value = 0x00;
    }
    value = value << 1; // Shift to match with mask
    return ads1120_write_reg_masked(value, REG_MASK_DRDY_MODE, CONFIG_REG3_ADDRESS);
}

esp_err_t ads1120_reset()
{
    return ads1120_send_command(CMD_RESET);
}

esp_err_t ads1120_start_sync()
{
    return ads1120_send_command(CMD_START_SYNC);
}

esp_err_t ads1120_power_down()
{
    return ads1120_send_command(CMD_PWRDWN);
}

esp_err_t ads1120_rdata()
{
    return ads1120_send_command(CMD_RDATA);
}