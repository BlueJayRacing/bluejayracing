#include <Arduino.h>

#include "component_template.hpp"

#include "ad5689.hpp"

AD5689 ad5689;
uint16_t level = 0xFFFF;
double voltage_val;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // comp_temp_func();

  // SPI.begin();

  ad5689_init_params_t init_params;
  init_params.t_sync_pin = 31;
  init_params.t_ldac_pin = 32;
  // init_params.t_rst_pin = 15;
  // init_params.t_spi_host = &SPI;

  ad5689.init(init_params);
}

void loop() {
  level += 100;

  voltage_val = 2.5 * level * pow(2, -15);

  ad5689.setLevel(A, level);
  Serial.println(level);
  Serial.println(voltage_val);
  delay(500);
}