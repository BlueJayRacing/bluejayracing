/*
  Current error checking implemented;
  - SD.open() will open an existing file with the specified name if possible, and create said file if the file does not exist
  - 
*/
#include "adc_test.hpp"
#include <SD.h>

#define CLCK_PIN_1 13
#define MISO_PIN_1 12
#define MOSI_PIN_1 11
#define CS_PIN_1 10
#define DATA_READY_PIN_1 17
#define ADC_RESET_THRESHHOLD 200

int count = 0;

xADS1120* ads1;
xADS1120* ads2;

void setup()
{
  Serial.begin(38400);
  Serial.println("Starting setup with 2 pins");

	ads1 = new xADS1120();
  ads1->begin(CLCK_PIN_1, MISO_PIN_1, MOSI_PIN_1, CS_PIN_1, DATA_READY_PIN_1, 0);
  Serial.println("ADS initialized");
}

void loop()
{
  uint16_t val = ads1->readADC();

  if (val == 0) {
    count++;
    if (count % ADC_RESET_THRESHHOLD == 0) {
      Serial.println("Device is broken, rebooting");
      ads1->reset();
    }
  }
  delay(1);
}