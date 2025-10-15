#include <Arduino.h>
#include "tlv493D.hpp"

Magnetometer* mag;

uint32_t iterations = 0;
uint64_t uS = 0; 
uint32_t old_iterations = 0;

void setup() {
  Wire.begin();
  mag = new Magnetometer();

  Serial.begin(115200);
  while (!Serial);  // wait for Serial monitor
  // digitalWrite(7, HIGH);
  // digitalWrite(8, HIGH);
  // for (int i = 0; i <= 0xFF; i++){
  mag->begin(Wire, TLV493D_ADDRESS1);
  Serial.println("Magnetometer ready");
  uS = micros();
}

void loop() {
  float x = 0, y = 0, z = 0;

  mag->readRawMag(x, y, z);

  // if (iterations % 200 == 0) {
  //   int64_t new_uS = micros();
  //   int64_t time_diff_us = (new_uS - uS);
  //   delay(5000);
  //   new_uS = micros();


  //   int32_t iter_diff = iterations - old_iterations;

  //   float hz = iter_diff / (time_diff_us / 1000000.);

  //   uS = new_uS;
  //   old_iterations = iterations;
    
  //   Serial.print(iter_diff);
  //   Serial.print(" iterations in ");
  //   Serial.print(time_diff_us);
  //   Serial.print(" us, Hz: ");
  //   Serial.println(hz);
  //   Serial.println(mag->getMeasurementDelay());
  //   // delay(5000);
  // }

  // delay(mag->getMeasurementDelay());

  
  iterations++;
  Serial.print("Magnetic Field: X=");
  Serial.print(x);
  Serial.print(" mT, Y=");
  Serial.print(y);
  Serial.print(" mT, Z=");
  Serial.print(z);
  Serial.println(" mT");

  // delay(500);  // 500ms delay between readings
}
