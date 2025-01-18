#include <Arduino.h>

#include "component_template.hpp"

#include "ad5689.hpp"

AD5689 ad5689;
uint16_t level = 0xFFFF;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // comp_temp_func();

  // SPI.begin();

  ad5689_init_params_t init_params;
  init_params.t_sync_pin = 13;
  init_params.t_ldac_pin = 9;
  init_params.t_rst_pin = 15;
  // init_params.t_spi_host = &SPI;

  ad5689.init(init_params);
}

void loop() {
  level += 100;

  ad5689.setLevel(B, level);
  Serial.println(level);
  delay(1000);
}