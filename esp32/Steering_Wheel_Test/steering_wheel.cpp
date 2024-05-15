#include "steering_wheel.h"

void steering_wheel::set_speed(int speed)
{
    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
        return;
    }
    this->speed = speed;
}

void steering_wheel::display_pixels_speed()
{
    int num_pixels_display = speed / MAX_SPEED * num_pixels;

    uint32_t green = pixels.Color(0, 50, 0, 0);
    uint32_t yellow = pixels.Color(25, 25, 0);
    uint32_t red = pixels.Color(50, 0, 0, 0);

    pixels.clear();
    for (int i = 0; i < num_pixels_display; i++)
    {
        if (i < 8)
        {
            pixels.setPixelColor(i, green);
        }
        else if (i < 12)
        {
            pixels.setPixelColor(i, yellow);
        }
        else
        {
            pixels.setPixelColor(i, red);
        }
    }
    pixels.show();
}

void steering_wheel::display_lcd_speed()
{
    lcd.clear();
    lcd.setCursor(2, 1);
    char message[10] = "Speed: ";

    std::stringstream strs;
    strs << speed;
    std::string temp_str = strs.str();
    char *char_type = (char *)temp_str.c_str();

    strcat(message, char_type);

    lcd.print(message);
}

void steering_wheel::write_message(char message[]) {
  lcd.clear();
  lcd.setCursor(2, 1);
  lcd.print(message);
}