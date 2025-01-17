#include <Arduino.h>
#include <pb_encode.h>

#include "component_template.hpp"

#include "max5719.hpp"
#include "ad5676.hpp"

MAX5719 max5719;
AD5676 ad5676;

void setup() {
  Serial.begin(115200);
  comp_temp_func();

  SPI.begin();
  // max5719.init(10, 9, &SPI);
  ad5676.init(10, 9, 8, &SPI);
}

void loop() {
  ad5676.setLevel(0, 0);
  delay(1000);
  for(int i = 0; i < 65536; i+=1000) {
    // max5719.setLevel(i);
    ad5676.setLevel(0, i); 
    Serial.println(i);
    delay(1000);
  }
  // int n = 1048575;
  // max5719.setLevel(n);
  // Serial.println(n);
  // delay(5000);
  
  // max5719.setLevel(0);
  // Serial.println(0);
  // delay(5000);
}