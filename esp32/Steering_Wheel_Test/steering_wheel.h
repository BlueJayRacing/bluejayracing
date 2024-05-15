#ifndef STEERING_WHEEL_H
#define STEERING_WHEEL_H

#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <sstream>
#include <string.h>

#define MAX_SPEED 40

class steering_wheel
{
public:
    steering_wheel(int pixel_pin, int num_pixels) : pixels(num_pixels, pixel_pin, NEO_RGB + NEO_KHZ800), lcd(0x27, 16, 3){this->num_pixels = num_pixels;}
    void set_speed(int speed);
    void display_pixels_speed();
    void display_lcd_speed();
    void write_message(char message[]);

private:
    int speed;
    int num_pixels;
    Adafruit_NeoPixel pixels;
    LiquidCrystal_I2C lcd;
};

#endif