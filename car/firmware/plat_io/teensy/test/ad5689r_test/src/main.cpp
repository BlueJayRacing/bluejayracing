#include <Arduino.h>

#include "component_template.hpp"

#include "ad5689r.hpp"

AD5689R ad5689r;
uint16_t level = 1;
double voltage_val;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // comp_temp_func();

  SPI.begin();

  ad5689r_init_param_t init_params;
  init_params.cs_pin = 10;
  init_params.ldac_pin = 32;
  init_params.clr_pin = 15;
  init_params.spi_host = &SPI;

  ad5689r.init(init_params);
}

void loop() {
  if (level == 0x8000) {
    level = 1;
  } else {
    level = level << 1;
  }

  voltage_val = 2.5 * level * pow(2, -15);

  ad5689r.setLevel(A, level);
  Serial.println(level);
  Serial.println(voltage_val);
  delay(4000);
}