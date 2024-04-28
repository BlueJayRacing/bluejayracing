/*
  This example shows how to get continous conversions out of the ADS1120.
  Lucas Etchezuri 30/06/2020
*/
#include "xADS1120.hpp"
#include <SD.h>
#include <TimeLib.h>

#define HALL_EFFECT_1_PIN 41 
#define HALL_EFFECT_2_PIN 40

#define CLCK_PIN_1 13
#define MISO_PIN_1 12
#define MOSI_PIN_1 11
#define CS_PIN_1 10
#define DATA_READY_PIN_1 17

#define CLCK_PIN_2 27
#define MISO_PIN_2 1
#define MOSI_PIN_2 26
#define CS_PIN_2 0
#define DATA_READY_PIN_2 20

enum Channel {
  HALL1 = 1,
  HALL2 = 2,
  LIMPOT1 = 3,
  LIMPOT2 = 4,
  AUX1 = 5,
  AUX2 = 6,
  GAUGE = 7,
  NO_CONNECT = 8
};

xADS1120* ads1;
xADS1120* ads2;

File file;
String s;
const int chip_select = BUILTIN_SDCARD;

int hall_counter_1 = 0;
int hall_counter_2 = 0;

bool hall_1_flag = false;
bool hall_2_flag = false;
bool adc_1_flag = false;
bool adc_2_flag = false;

#define WRITE_TIME 10000000
uint32_t diff;
uint32_t now1;

void setup()
{
  attachInterrupt(digitalPinToInterrupt(DATA_READY_PIN_1), adc_interrupt_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(DATA_READY_PIN_2), adc_interrupt_2, FALLING);
  attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_1_PIN), hall_effect_interrupt_1, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_2_PIN), hall_effect_interrupt_2, RISING);

  Serial.begin(38400);
  Serial.println("Starting setup with 2 pins");

	ads1 = new xADS1120();
  ads1->begin(CLCK_PIN_1, MISO_PIN_1, MOSI_PIN_1, CS_PIN_1, DATA_READY_PIN_1, 0);

  ads2 = new xADS1120();
  ads2->begin(CLCK_PIN_2, MISO_PIN_2, MOSI_PIN_2, CS_PIN_2, DATA_READY_PIN_2, 1);

  while (!SD.begin(chip_select)) {
    Serial.println("could not open sd");
    delay(10);
  }

  setSyncProvider(getTeensy3Time);
  
  s = String(hour())+ "_" + String(minute())+"cali_endurance.txt";
  file = SD.open(s.c_str(), FILE_WRITE);
  now1 = micros();
  diff = micros() - now1;
}

void loop()
{

  if (hall_1_flag) {
    hall_1_flag = false;
    record(micros(), Channel::HALL1, hall_counter_1);
  }

  if (hall_2_flag) {
    hall_2_flag = false;
    record(micros(), Channel::HALL2, hall_counter_2);
  }

  if (adc_1_flag) {
    adc_1_flag = false;
    get_adc1_values();
  }

  if (adc_2_flag) {
    adc_2_flag = false;
    get_adc2_values();
  }

  diff = micros() - now1;
  if (diff > WRITE_TIME) {
    file.close();
    file = SD.open(s.c_str(), FILE_WRITE);
    now1 = micros();
  }
}

void adc_interrupt_1() {
  adc_1_flag = true;
}

void adc_interrupt_2() {
  adc_2_flag = true;
}

void hall_effect_interrupt_1() {
  hall_counter_1++;
  hall_1_flag = true;
}


void hall_effect_interrupt_2() {
  hall_counter_2++;
  hall_2_flag = true;
}


void get_adc1_values() {
  static int adc1_mux = 0x8;
  int value = ads1->readADC();
  uint32_t now = micros();

  switch (adc1_mux) {
    case 0x8:
      record(now, Channel::LIMPOT1, value);
      adc1_mux = 0x9;
      break;
    case 0x9:
      record(now, Channel::GAUGE, value);
      adc1_mux = 0xA;
      break;
    case 0xA:
      record(now, Channel::NO_CONNECT, value);
      adc1_mux = 0x8;
  }
  ads1->setMultiplexer(adc1_mux);
}

void get_adc2_values() {
  static int adc2_mux = 0x8;
  int value = ads2->readADC();
  uint32_t now = micros();

  switch (adc2_mux) {
    case 0x8:
      record(now, Channel::LIMPOT2, value);
      adc2_mux = 0x9;
      break;
    case 0x9:
      record(now, Channel::AUX2, value);
      adc2_mux = 0xA;
      break;
    case 0xA:
      record(now, Channel::AUX1, value);
      adc2_mux = 0x8;
  }
    ads2->setMultiplexer(adc2_mux);
}

void record(uint32_t time, Channel channel, uint16_t value) {
  file.print(time);
  file.print(F(","));
  file.print(channel);
  file.print(F(","));
  file.println(value);
}

time_t getTeensy3Time()
{
return Teensy3Clock.get();
}