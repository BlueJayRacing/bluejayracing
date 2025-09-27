#include <Arduino.h>
#include "magnetometer.hpp"

Magnetometer* mag;

void setup() {
  Wire1.begin();
  mag = new Magnetometer(Wire1, TLx493D_IIC_ADDR_A0_e);

  Serial.begin(115200);
  while (!Serial);  // wait for Serial monitor

  if (!mag->begin()) {
    Serial.println("Failed to initialize magnetometer");
    while (1); // halt
  }
  Serial.println("Magnetometer ready");
}

void loop() {
  int16_t x = 0, y = 0, z = 0;

  if (mag->readRawMag(x, y, z)) {
    Serial.print("Magnetic Field: X=");
    Serial.print(x);
    Serial.print(" mT, Y=");
    Serial.print(y);
    Serial.print(" mT, Z=");
    Serial.print(z);
    Serial.println(" mT");
  } else {
    Serial.println("Read failed");
  }

  delay(500);  // 500ms delay between readings
}
