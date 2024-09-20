#include <Arduino.h>

#include "component_template.hpp"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  comp_temp_func();
}

void loop() {
  // put your main code here, to run repeatedly:
}
