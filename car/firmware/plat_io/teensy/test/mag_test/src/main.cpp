#include <Arduino.h>
#include "magnetometer.hpp"

Magnetometer* mag;

void setup() {
  Wire1.begin();
  mag = new Magnetometer();

  Serial.begin(115200);
  while (!Serial);  // wait for Serial monitor
  // digitalWrite(7, HIGH);
  // digitalWrite(8, HIGH);
  for (int i = 0; i <= 0xFF; i++){
    if (!mag->begin(Wire1, i)) {
      Serial.println("Failed to initialize magnetometer: " + String(i));
    } else {
      Serial.println(String(i));
      break;
    }
  }
  Serial.println("Magnetometer ready");
}

void loop() {
  int16_t x = 0, y = 0, z = 0;

  mag->readRawMag(x, y, z);
  Serial.print("Magnetic Field: X=");
  Serial.print(x);
  Serial.print(" mT, Y=");
  Serial.print(y);
  Serial.print(" mT, Z=");
  Serial.print(z);
  Serial.println(" mT");

  delay(500);  // 500ms delay between readings
}
