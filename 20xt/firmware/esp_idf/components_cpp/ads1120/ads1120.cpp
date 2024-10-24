#include "esp_log.h"

#include "ads1120.hpp"

static const char* TAG = "ads1120";

#define ADS_SPI_LOCK_TIMEOUT 10
#define ADS_SPI_CLOCK_SPEED_HZ 1 * 1000 * 1000
#define ADS_SPI_MODE 1
#define ADS_CS_EN_PRE_WAIT_CYCLES 2
#define ADS_CS_EN_POST_WAIT_CYCLES 0
#define ADS_SPI_INPUT_DELAY_NS 0

ADS1120::ADS1120()
{
}

esp_err_t ADS1120::sendCommand(uint8_t t_command)
{
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(spi_transaction_t));

  t.flags = SPI_TRANS_USE_TXDATA;
  t.tx_data[0] = t_command;
  t.length = 1 * 8;

  ret = spi_device_polling_transmit(m_spi_dev, &t); // Transmit!
  if (ret != ESP_OK)
  {
    return ret;
  }

  return ret;
}

esp_err_t ADS1120::writeRegister(uint8_t t_address, uint8_t t_value)
{
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(spi_transaction_t));

  t.flags = SPI_TRANS_USE_TXDATA;
  t.tx_data[0] = (ADS1120_CMD_WREG | (t_address << 2));
  t.tx_data[1] = t_value;
  t.length = 2 * 8; // 2 bytes

  ret = spi_device_polling_transmit(m_spi_dev, &t); // Transmit!
  if (ret != ESP_OK)
  {
    return ret;
  }

  return ret;
}

esp_err_t ADS1120::readRegister(uint8_t t_address, uint8_t *t_data)
{
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(spi_transaction_t));

  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.length = 2 * 8;   // 2 bytes
  t.rxlength = 2 * 8; // 2 bytes
  t.tx_data[0] = (ADS1120_CMD_RREG | (t_address << 2));
  t.tx_data[1] = ADS1120_SPI_MASTER_DUMMY;

  ret = spi_device_polling_transmit(m_spi_dev, &t); // Transmit!
  if (ret != ESP_OK)
  {
    return ret;
  }

  *t_data = t.rx_data[1];
  return ret;
}

esp_err_t ADS1120::init(gpio_num_t t_cs_pin, gpio_num_t t_drdy_pin, spi_host_device_t t_spi_host)
{
  // Set pins up
  m_cs_pin = t_cs_pin;
  m_drdy_pin = t_drdy_pin;

  gpio_set_direction(m_cs_pin, GPIO_MODE_OUTPUT);
  gpio_set_direction(m_drdy_pin, GPIO_MODE_INPUT);

  spi_device_interface_config_t devcfg;
  memset(&devcfg, 0, sizeof(spi_device_interface_config_t));

  devcfg.clock_source = SPI_CLK_SRC_DEFAULT;
  devcfg.mode = ADS_SPI_MODE;
  devcfg.cs_ena_pretrans = ADS_CS_EN_PRE_WAIT_CYCLES;
  devcfg.cs_ena_posttrans = ADS_CS_EN_POST_WAIT_CYCLES;
  devcfg.clock_speed_hz = ADS_SPI_CLOCK_SPEED_HZ;
  devcfg.input_delay_ns = ADS_SPI_INPUT_DELAY_NS;
  devcfg.spics_io_num = m_cs_pin; // CS pin
  devcfg.flags = 0;
  devcfg.queue_size = 1;

  esp_err_t err = spi_bus_add_device(t_spi_host, &devcfg, &m_spi_dev);
  if (err != ESP_OK)
  {
    return err;
  }

  ESP_LOGI(TAG, "Added SPI Device to Bus\n");

  vTaskDelay(5);

  reset();
  vTaskDelay(5);
  return startSync(); // Send start/sync for continuous conversion mode
}

bool ADS1120::isDataReady()
{
  return !gpio_get_level(m_drdy_pin);
}

esp_err_t ADS1120::readADC(uint16_t *t_data)
{
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(spi_transaction_t));

  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  memset(t.tx_data, ADS1120_SPI_MASTER_DUMMY, 2);

  t.length = 2 * 8;   // 2 bytes
  t.rxlength = 2 * 8; // 2 bytes

  ret = spi_device_polling_transmit(m_spi_dev, &t); // Transmit!
  if (ret != ESP_OK)
  {
    return ret;
  }
  
  *t_data = t.rx_data[0];
  *t_data = (*t_data << 8) | t.rx_data[1];
  return ret;
}

esp_err_t ADS1120::writeRegisterMasked(uint8_t t_value, uint8_t t_mask, uint8_t t_address)
{
  // Escribe un valor en el registro, aplicando la mascara para tocar unicamente los bits necesarios.
  // No realiza el corrimiento de bits (shift), hay que pasarle ya el valor corrido a la posicion correcta

  // Leo el contenido actual del registro
  uint8_t register_contents;
  readRegister(t_address, &register_contents);

  // Cambio bit aa bit la mascara (queda 1 en los bits que no hay que tocar y 0 en los bits a modificar)
  // Se realiza un AND co el contenido actual del registro.  Quedan "0" en la parte a modificar
  register_contents = register_contents & ~t_mask;

  // se realiza un OR con el valor a cargar en el registro.  Ojo, valor debe estar en el posicion (shitf) correcta
  register_contents = register_contents | t_value;

  // Escribo nuevamente el registro
  return writeRegister(t_address, register_contents);
}

esp_err_t ADS1120::setMultiplexer(uint8_t t_value)
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
  if (t_value > 0x0E)
  {
    t_value = 0x00;
  }
  t_value = t_value << 4; // Shift to match with mask
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_MUX, ADS1120_CONFIG_REG0_ADDRESS);
}

esp_err_t ADS1120::setGain(uint8_t t_gain)
{
  /* Sets ADC gain. Possible values are 1, 2, 4, 8, 16, 32, 64, 128. */
  uint8_t value = 0x00;
  switch (t_gain)
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
  return writeRegisterMasked(value, ADS1120_REG_MASK_GAIN, ADS1120_CONFIG_REG0_ADDRESS);
}

esp_err_t ADS1120::setPGAbypass(bool t_value)
{
  /* Bypasses the PGA if true.
     PGA can only be disabled for gains 1, 2, 4.
  */
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_PGA_BYPASS, ADS1120_CONFIG_REG0_ADDRESS);
}

esp_err_t ADS1120::setDataRate(uint8_t t_value)
{
  /* Sets the data rate for the ADC. See table 18 in datasheet for datarates
     in various operating modes. */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (t_value > 0x07)
  {
    t_value = 0x00;
  }
  t_value = t_value << 5; // Shift to match with mask
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_DATARATE, ADS1120_CONFIG_REG1_ADDRESS);
}

esp_err_t ADS1120::setOpMode(uint8_t t_value)
{
  /* Sets the ADC operating mode:
     0 - Normal mode
     1 - Duty-cycle mode
     2 - Turbo mode
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (t_value > 0x02)
  {
    t_value = 0x00;
  }
  t_value = t_value << 3; // Shift to match with mask
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_OP_MODE, ADS1120_CONFIG_REG1_ADDRESS);
}

esp_err_t ADS1120::setConversionMode(uint8_t t_value)
{
  /* Sets the ADC conversion mode.
     0 - Single shot mode
     1 - continuous conversion mode
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (t_value > 0x01)
  {
    t_value = 0x00;
  }
  t_value = t_value << 2; // Shift to match with mask
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_CONV_MODE, ADS1120_CONFIG_REG1_ADDRESS);
}

esp_err_t ADS1120::setTemperatureMode(uint8_t t_value)
{
  /* Controls the state of the internal temperature sensor.
     0 - Disables temperature sensor
     1 - Enables temperature sensor
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (t_value > 0x01)
  {
    t_value = 0x00;
  }
  t_value = t_value << 1; // Shift to match with mask
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_TEMP_MODE, ADS1120_CONFIG_REG1_ADDRESS);
}

esp_err_t ADS1120::setBurnoutCurrentSources(bool t_value)
{
  /* Turns the 10uA burn-out current sources on or off. */
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_BURNOUT_SOURCES, ADS1120_CONFIG_REG1_ADDRESS);
}

esp_err_t ADS1120::setVoltageRef(uint8_t t_value)
{
  /* Sets the voltage reference used by the ADC.
     0 - Internal 2.048 V
     1 - External on REFP0 and REFN0 inputs
     2 - External on AIN0/REFP1 and AIN3/REFN1 inputs
     3 - Use analog supply as reference
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (t_value > 0x03)
  {
    t_value = 0x00;
  }
  t_value = t_value << 6; // Shift to match with mask
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_VOLTAGE_REF, ADS1120_CONFIG_REG2_ADDRESS);
}

esp_err_t ADS1120::setFIR(uint8_t t_value)
{
  /* Controls the FIR filter on the ADC.
     0 - No 50 or 60 Hz rejection
     1 - Both 50 and 60 Hz rejection
     2 - 50 Hz rejection
     3 - 60 Hz rejection
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (t_value > 0x03)
  {
    t_value = 0x00;
  }
  t_value = t_value << 4; // Shift to match with mask
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_FIR_CONF, ADS1120_CONFIG_REG2_ADDRESS);
}

esp_err_t ADS1120::setPowerSwitch(uint8_t t_value)
{
  /* Configures behavior of low-side switch between AIN3/REFN1 and AVSS.
     0 - Always open
     1 - Automatically closes when START/SYNC command is sent and opens when
         POWERDOWN command is issues.
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (t_value > 0x01)
  {
    t_value = 0x00;
  }
  t_value = t_value << 3; // Shift to match with mask
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_PWR_SWITCH, ADS1120_CONFIG_REG2_ADDRESS);
}

esp_err_t ADS1120::setIDACcurrent(uint8_t t_value)
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
  if (t_value > 0x07)
  {
    t_value = 0x00;
  }
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_IDAC_CURRENT, ADS1120_CONFIG_REG2_ADDRESS);
}

esp_err_t ADS1120::setIDAC1routing(uint8_t t_value)
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
  if (t_value > 0x06)
  {
    t_value = 0x00;
  }
  t_value = t_value << 5; // Shift to match with mask
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_IDAC1_ROUTING, ADS1120_CONFIG_REG3_ADDRESS);
}

esp_err_t ADS1120::setIDAC2routing(uint8_t t_value)
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
  if (t_value > 0x06)
  {
    t_value = 0x00;
  }
  t_value = t_value << 2; // Shift to match with mask
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_IDAC2_ROUTING, ADS1120_CONFIG_REG3_ADDRESS);
}

esp_err_t ADS1120::setDRDYmode(uint8_t t_value)
{
  /* Controls the behavior of the DOUT/DRDY pin when new data are ready.
     0 - Only the dedicated DRDY pin is used  (Default)
     1 - Data ready indicated on DOUT/DRDY and DRDY
 */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (t_value > 0x01)
  {
    t_value = 0x00;
  }
  t_value = t_value << 1; // Shift to match with mask
  return writeRegisterMasked(t_value, ADS1120_REG_MASK_DRDY_MODE, ADS1120_CONFIG_REG3_ADDRESS);
}

esp_err_t ADS1120::reset()
{
  return sendCommand(ADS1120_CMD_RESET);
}

esp_err_t ADS1120::startSync()
{
  return sendCommand(ADS1120_CMD_START_SYNC);
}

esp_err_t ADS1120::powerDown()
{
  return sendCommand(ADS1120_CMD_PWRDWN);
}

esp_err_t ADS1120::rdata()
{
  return sendCommand(ADS1120_CMD_RDATA);
}