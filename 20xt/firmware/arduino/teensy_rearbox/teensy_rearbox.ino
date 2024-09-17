#include <limits>

#include <ADS1115_lite.h>
#include <TimeLib.h>
#include <SD.h>

const int chip_select = BUILTIN_SDCARD;

File file;
String curr_file_path;
int major_index;
int minor_index;
int orig_minute;

#define FILE_TIME 180000000
uint32_t file_diff;
uint32_t file_now;

ADS1115_lite adcs[4];
int16_t Raw;

bool hall1_flag = false;
bool hall2_flag = false;
int hall1_count = 0;
int hall2_count = 0;

enum Channel {
  HALL1 = 0,
  HALL2 = 1,
  LIMPOT1 = 2,
  LIMPOT2 = 3,
  LIMPOT3 = 4,
  LIMPOT4 = 5
};

void setup(void) {
  orig_minute = minute();
  Serial.begin(9600);
  Serial.println(F("Set Serial baud rate to 250 000"));
  Serial.flush();  // wait for send buffer to empty
  delay(2);        // let last character be sent
  Serial.end();    // close serial
  Serial.begin(250000);
  Serial.println();

  while (!SD.begin(chip_select)) {
    Serial.println("could not open sd");
    delay(10);
  }

  major_index = read_meta_file_index() + 1;
  write_meta_file_index(major_index);
  minor_index = 0;

  curr_file_path = generate_file_path(major_index, minor_index);
  file = SD.open(curr_file_path.c_str(), FILE_WRITE);

  for (int i = 0; i < 4; i++) {
    adcs[i] = *(new ADS1115_lite(0x48 + i));
    adcs[i].setGain(ADS1115_REG_CONFIG_PGA_6_144V);
    adcs[i].setSampleRate(ADS1115_REG_CONFIG_DR_860SPS);
    adcs[i].setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
    adcs[i].triggerConversion();
  }

  attachInterrupt(digitalPinToInterrupt(13), hall_interrupt_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(41), hall_interrupt_2, FALLING);

  file_now = micros();
  file_diff = micros();
}

void loop(void) {
  for (int i = 0; i < 4; i++) {
    int val = adcs[i].getConversion();
    record(millis(), micros(), (Channel) (i + 2), val); 
  }

  if (hall1_flag) {
    hall1_flag = false;
    record(millis(), micros(), HALL1, hall1_count);
  }

  if (hall2_flag) {
    hall2_flag = false;
    record(millis(), micros(), HALL2, hall2_count);
  }

  file_diff = micros() - file_now;

  if (file_diff > FILE_TIME) {
    file.close();

    while (SD.exists(curr_file_path.c_str()))
    {
      curr_file_path = generate_file_path(major_index, ++minor_index);
    }

    file = SD.open(curr_file_path.c_str(), FILE_WRITE);
    file_now = micros();
  }

  delayMicroseconds(100);
}

void record(uint32_t time_milli, uint32_t time_micro, Channel channel, int32_t value) {
  file.print(time_milli);
  file.print(F(","));
  file.print(time_micro);
  file.print(F(","));
  file.print(channel);
  file.print(F(","));
  file.print(value);
  file.println(F(","));
}

void hall_interrupt_1() {
  hall1_count++;
  hall1_flag = true;
}

void hall_interrupt_2() {
  hall2_count++;
  hall2_flag = true;
}

String generate_file_path(int major_index, int minor_index)
{
  String major_string = String(major_index);
  while (major_string.length() < 3) {
    major_string = "0" + major_string;
  }

  String minor_string = String(minor_index);
  while (minor_string.length() < 3) {
    minor_string = "0" + minor_string;
  }

  return major_string + "_" + minor_string + "_michigan.txt";
}

int read_meta_file_index(void)
{
  if (!SD.exists("metadata.txt"))
  {
    write_meta_file_index(0);
    return -1;
  }

  File meta_file = SD.open("metadata.txt", FILE_READ);
  String line = meta_file.readStringUntil('\n');
  int index = line.toInt();
  meta_file.close();
  return index;
}

void write_meta_file_index(int index)
{
  File meta_file = SD.open("metadata.txt", FILE_WRITE_BEGIN);
  meta_file.println(index);
  meta_file.close();
}