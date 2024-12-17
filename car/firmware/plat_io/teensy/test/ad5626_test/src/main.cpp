#include <Arduino.h>

#include "component_template.hpp"

#include "ad5626.hpp"

AD5626 ad5626;
uint16_t level = 0xFFFF;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  comp_temp_func();

  SPI.begin();

  ad5626.init(26, 25, -1, &SPI);
}

void loop() {
  level += 100;

  ad5626.setLevel(level);
  delay(1000);
}
