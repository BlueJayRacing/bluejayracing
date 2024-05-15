#include "steering_wheel.h"

steering_wheel wheel(15, 16);
int speed = 0;

void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(34), interrupt1, RISING);
  attachInterrupt(digitalPinToInterrupt(33), interrupt2, RISING);
  attachInterrupt(digitalPinToInterrupt(32), interrupt3, RISING);
  attachInterrupt(digitalPinToInterrupt(31), interrupt4, RISING);
}

void loop() {
  speed++;
  wheel.set_speed(speed);
  wheel.display_lcd_speed();
  wheel.display_pixels_speed();

  if (speed > 40) {
    speed = 0;
  }
  delay(100);
}

void interrupt1() {
  wheel.write_message("Switch 1 pressed");
}

void interrupt2() {
  wheel.write_message("Switch 2 pressed");
}

void interrupt3() {
  wheel.write_message("Switch 3 pressed");
}

void interrupt4() {
  wheel.write_message("Switch 4 pressed");
}
