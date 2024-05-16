#include "Arduino.h"
#include "TeensyADS1120.h"
#include "SPI.h"

TeensyADS1120::TeensyADS1120()
{
}

void TeensyADS1120::writeRegister(uint8_t address, uint8_t value)
{
  if (spi_num == 0) {
    SPI.transfer(CMD_WREG | (address << 2));
    SPI.transfer(value);
  } else {
    SPI1.transfer(CMD_WREG | (address << 2));
    SPI1.transfer(value);
  }
  delayMicroseconds(10);
}

uint8_t TeensyADS1120::readRegister(uint8_t address)
{
  uint8_t data;
  if (spi_num == 0) {
    SPI.transfer(CMD_RREG | (address << 2));
    data = SPI.transfer(SPI_MASTER_DUMMY);
  } else {
    SPI1.transfer(CMD_RREG | (address << 2));
    data = SPI1.transfer(SPI_MASTER_DUMMY);
  }
  delayMicroseconds(1);
  return data;
}

void TeensyADS1120::begin(uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin, uint8_t drdy_pin, int spi_num)
{
  // Set pins up
  ADS1120_CS_PIN = cs_pin;
  ADS1120_DRDY_PIN = drdy_pin;
  ADS1120_CLK_PIN = clk_pin;
  ADS1120_MISO_PIN = miso_pin;
  ADS1120_MOSI_PIN = mosi_pin;

  // Configure the SPI interface (CPOL=0, CPHA=1)
  if (spi_num == 0) {
    SPI.begin();
    SPI.setDataMode(SPI_MODE1);
    this->spi_num = 0;
  } else {
    SPI1.begin();
    SPI1.setDataMode(SPI_MODE1);
    this->spi_num = 1;
  }
  
  // Configure chip select as an output
  pinMode(ADS1120_CS_PIN, OUTPUT);
  // Configure DRDY as as input
  pinMode(ADS1120_DRDY_PIN, INPUT);

  digitalWrite(ADS1120_CS_PIN, LOW); // Set CS Low
  delayMicroseconds(1);              
  reset();                           
  delayMicroseconds(1);
  ; // Delay a minimum of 50 us + 32 * tclk

  startSync();                        // Send start/sync for continuous conversion mode
}

bool TeensyADS1120::isDataReady()
{
  if (digitalRead(ADS1120_DRDY_PIN) == HIGH)
  {
    return false;
  }
  return true;
}

uint16_t TeensyADS1120::read()
{
  uint16_t adcVal;
  if (spi_num == 0) {
    adcVal = SPI.transfer(SPI_MASTER_DUMMY);
    adcVal = (adcVal << 8) | SPI.transfer(SPI_MASTER_DUMMY);
  } else {
    adcVal = SPI1.transfer(SPI_MASTER_DUMMY);
    adcVal = (adcVal << 8) | SPI1.transfer(SPI_MASTER_DUMMY);
  }
  delayMicroseconds(1);
  return adcVal;
}

byte *TeensyADS1120::readADC_Array()
{
  static byte dataarray[2];
  for (int x = 0; x < 2; x++)
  {
    if (spi_num == 0) {
      dataarray[x] = SPI.transfer(SPI_MASTER_DUMMY);
    } else {
      dataarray[x] = SPI1.transfer(SPI_MASTER_DUMMY);      
    }
  }
  delayMicroseconds(1);
  return dataarray;
}

//Single Conversion read modes
int TeensyADS1120::readADC_Single()
{
  if (spi_num == 0) {
    SPI.transfer(0x08);
  } else {
    SPI1.transfer(0x08);
  }
  delayMicroseconds(1);

  while (digitalRead(ADS1120_DRDY_PIN) == HIGH)
  {
    // Espera a que DRDY se ponga en nivel bajo. Esto es un riesgo porque pude quedar bloqueado el codigo aca.
    // Se deberia poner un timeout configurable en el metodo de begin y devolver un error si no responde 
  }

  int adcVal;

  if (spi_num == 0) {
    adcVal = SPI.transfer(SPI_MASTER_DUMMY);
    adcVal = (adcVal << 8) | SPI.transfer(SPI_MASTER_DUMMY);
  } else {
    adcVal = SPI1.transfer(SPI_MASTER_DUMMY);
    adcVal = (adcVal << 8) | SPI1.transfer(SPI_MASTER_DUMMY);    
  }
  delayMicroseconds(1);

  return adcVal;
}

byte *TeensyADS1120::readADC_SingleArray()
{
  if (spi_num == 0) {
    SPI.transfer(0x08);
  } else {
    SPI1.transfer(0x08);
  }
  delayMicroseconds(1);

  while (digitalRead(ADS1120_DRDY_PIN) == HIGH)
  {
    // Espera a que DRDY se ponga en nivel bajo. Esto es un riesgo porque pude quedar bloqueado el codigo aca.
    // Se deberia poner un timeout configurable en el metodo de begin y devolver un error si no responde
  }

  static byte dataarray[2];
  for (int x = 0; x < 2; x++)
  {
    if (spi_num == 0) {
      dataarray[x] = SPI.transfer(SPI_MASTER_DUMMY);
    } else {
      dataarray[x] = SPI1.transfer(SPI_MASTER_DUMMY);      
    }
  }
  delayMicroseconds(1);

  return dataarray;
}

void TeensyADS1120::sendCommand(uint8_t command)
{
  SPI.transfer(command);
}

void TeensyADS1120::writeRegisterMasked(uint8_t value, uint8_t mask, uint8_t address)
{
  // Escribe un valor en el registro, aplicando la mascara para tocar unicamente los bits necesarios.
  // No realiza el corrimiento de bits (shift), hay que pasarle ya el valor corrido a la posicion correcta
  
  // Leo el contenido actual del registro
  uint8_t register_contents = readRegister(address);

  // Cambio bit aa bit la mascara (queda 1 en los bits que no hay que tocar y 0 en los bits a modificar)
  // Se realiza un AND co el contenido actual del registro.  Quedan "0" en la parte a modificar
  register_contents = register_contents & ~mask;

  // se realiza un OR con el valor a cargar en el registro.  Ojo, valor debe estar en el posicion (shitf) correcta
  register_contents = register_contents | value;

  // Escribo nuevamente el registro
  writeRegister(address, register_contents);
}

void TeensyADS1120::setMultiplexer(uint8_t value)
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
  writeRegisterMasked(value, REG_MASK_MUX, CONFIG_REG0_ADDRESS);
}

void TeensyADS1120::setGain(uint8_t gain)
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
  writeRegisterMasked(value, REG_MASK_GAIN, CONFIG_REG0_ADDRESS);
}

void TeensyADS1120::setPGAbypass(bool value)
{
  /* Bypasses the PGA if true.
     PGA can only be disabled for gains 1, 2, 4.
  */
  writeRegisterMasked(value, REG_MASK_PGA_BYPASS, CONFIG_REG0_ADDRESS);
}

void TeensyADS1120::setDataRate(uint8_t value)
{
  /* Sets the data rate for the ADC. See table 18 in datasheet for datarates
     in various operating modes. */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x07)
  {
    value = 0x00;
  }
  value = value << 5; // Shift to match with mask
  writeRegisterMasked(value, REG_MASK_DATARATE, CONFIG_REG1_ADDRESS);
}

void TeensyADS1120::setOpMode(uint8_t value)
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
  writeRegisterMasked(value, REG_MASK_OP_MODE, CONFIG_REG1_ADDRESS);
}

void TeensyADS1120::setConversionMode(uint8_t value)
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
  writeRegisterMasked(value, REG_MASK_CONV_MODE, CONFIG_REG1_ADDRESS);
}

void TeensyADS1120::setTemperatureMode(uint8_t value)
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
  writeRegisterMasked(value, REG_MASK_TEMP_MODE, CONFIG_REG1_ADDRESS);
}

void TeensyADS1120::setBurnoutCurrentSources(bool value)
{
  /* Turns the 10uA burn-out current sources on or off. */
  writeRegisterMasked(value, REG_MASK_BURNOUT_SOURCES, CONFIG_REG1_ADDRESS);
}

void TeensyADS1120::setVoltageRef(uint8_t value)
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
  writeRegisterMasked(value, REG_MASK_VOLTAGE_REF, CONFIG_REG2_ADDRESS);
}

void TeensyADS1120::setFIR(uint8_t value)
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
  writeRegisterMasked(value, REG_MASK_FIR_CONF, CONFIG_REG2_ADDRESS);
}

void TeensyADS1120::setPowerSwitch(uint8_t value)
{
  /* Configures behavior of low-side switch between AIN3/REFN1 and AVSS.
     0 - Always open
     1 - Automatically closes when START/SYNC command is sent and opens when
         POWERDOWN command is issues.
  */
  // Make sure the value is in the valid range. Otherwise set to 0x00
  if (value > 0x01)
  {
    value = 0x00;
  }
  value = value << 3; // Shift to match with mask
  writeRegisterMasked(value, REG_MASK_PWR_SWITCH, CONFIG_REG2_ADDRESS);
}

void TeensyADS1120::setIDACcurrent(uint8_t value)
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
  writeRegisterMasked(value, REG_MASK_IDAC_CURRENT, CONFIG_REG2_ADDRESS);
}

void TeensyADS1120::setIDAC1routing(uint8_t value)
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
  writeRegisterMasked(value, REG_MASK_IDAC1_ROUTING, CONFIG_REG3_ADDRESS);
}

void TeensyADS1120::setIDAC2routing(uint8_t value)
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
  writeRegisterMasked(value, REG_MASK_IDAC2_ROUTING, CONFIG_REG3_ADDRESS);
}

void TeensyADS1120::setDRDYmode(uint8_t value)
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
  writeRegisterMasked(value, REG_MASK_DRDY_MODE, CONFIG_REG3_ADDRESS);
}

void TeensyADS1120::reset()
{
  sendCommand(CMD_RESET);
}

void TeensyADS1120::startSync()
{
  sendCommand(CMD_START_SYNC);
}

void TeensyADS1120::powerDown()
{
  sendCommand(CMD_PWRDWN);
}

void TeensyADS1120::rdata()
{
  sendCommand(CMD_RDATA);
}