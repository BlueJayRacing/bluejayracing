#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#include "component_template.hpp"
#include "ad5626.hpp"

AD5626 ad5626;
uint16_t level = 0xFFFF;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  comp_temp_func();

  ad5626.init(26, 25, -1);
}

void loop() {
  level += 1000;
  Serial.println("Set new level");
  ad5626.setLevel(level);
  delay(1000);
}
