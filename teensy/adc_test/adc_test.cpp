#include "adc_test.hpp"

void xADS1120::begin(int clk_pin, int miso_pin, int mosi_pin, int cs_pin, int drdy_pin, int spi_num) {
  adc = new TeensyADS1120();
  adc->begin(clk_pin, miso_pin, mosi_pin, cs_pin, drdy_pin, spi_num);
  adc->setGain(1);				 //Set gain 1.  Possible values are 1, 2, 4, 8, 16, 32, 64, 128.
	adc->setOpMode(0x02);		 // Set Turbo Mode
	adc->setDataRate(0x06);		 // Set Data rate 110.
	adc->setConversionMode(0x01); // 1=Continous Mode
	adc->setMultiplexer(0x09);
  adc->setVoltageRef(1);
}

uint16_t xADS1120::readADC() {
  return adc->read();
}

int xADS1120::readADCSingle() {
  return adc->readADC_Single();
}

void xADS1120::setMultiplexer(int mux) {
  adc->setMultiplexer(mux);
}

uint8_t xADS1120::readRegister(uint8_t address) {
  return adc->readRegister(address);
}

void xADS1120::reset() {
  adc->reset();
}
