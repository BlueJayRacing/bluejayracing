#include "src/xADS1120/xADS1120.hpp"
#include <SD.h>
#include <TimeLib.h>
#include <data_logging.h>

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

#define ADC_FAIL_THRESHHOLD 1000

uint16_t last_adc1_fail = 0;
int adc1_fail_count = 0;

uint16_t last_adc2_fail = 0;
int adc2_fail_count = 0;

enum Channel {
  HALL1 = 0,
  HALL2 = 1,
  LIMPOT1 = 2,
  LIMPOT2 = 3,
  AUX1 = 4,
  AUX2 = 5,
};

xADS1120* ads1;
xADS1120* ads2;

BajaDataLogging::BajaDataWriter* writer;
std::vector<uint8_t> channel_ids;

int hall_counter_1 = 0;
int hall_counter_2 = 0;

bool hall_1_flag = false;
bool hall_2_flag = false;
bool adc_1_flag = false;
bool adc_2_flag = false;

#define ADC_TIME 500000000
uint64_t adc_diff;
uint64_t adc_now;

void setup() {
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

  setSyncProvider(getTeensy3Time);

  std::string s = std::to_string(hour()) + "_" + std::to_string(minute()) + "cali_endurance.txt";
  writer = new BajaDataLogging::BajaDataWriter(s);

  channel_ids.push_back(writer->add_channel("Hall1", "tick", "1", "micros", std::to_string(sizeof(uint16_t))));
  channel_ids.push_back(writer->add_channel("Hall2", "tick", "1", "micros", std::to_string(sizeof(uint16_t))));
  channel_ids.push_back(writer->add_channel("LIMPOT1", "unit", "1", "micros", std::to_string(sizeof(uint16_t))));
  channel_ids.push_back(writer->add_channel("LIMPOT2", "unit", "1", "micros", std::to_string(sizeof(uint16_t))));
  channel_ids.push_back(writer->add_channel("AUX1", "unit", "1", "micros", std::to_string(sizeof(uint16_t))));
  channel_ids.push_back(writer->add_channel("AUX2", "unit", "1", "micros", std::to_string(sizeof(uint16_t))));

  Serial.println("File opened");

  adc_now = micros();
  adc_diff = micros() - adc_now;
}

void loop() {
  
  if (hall_1_flag) {
    Serial.print("Hall 1: ");
    Serial.println(hall_counter_1);
    hall_1_flag = false;
    record(micros(), Channel::HALL1, hall_counter_1);
  }
  
  if (hall_2_flag) {
    Serial.println("Hall 2");
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

  adc_diff = micros() - adc_now;
  if (adc_diff > ADC_TIME) {
    ads1->reset();
    ads2->reset();
    adc_now = micros();
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
  //ads1->setMultiplexer(adc1_mux);

  if (value == 0 || value == 0xff) {
    adc1_fail_count++;
    if (adc1_fail_count % ADC_FAIL_THRESHHOLD == 0) {
      Serial.println("ADC 1 failed: resetting");
      ads1->reset();
    }
    return;
  }

  adc1_fail_count = 0;

  uint64_t now = micros();

  switch (adc1_mux) {
    case 0x8:
      record(now, Channel::LIMPOT1, value);
      break;
  }
}

void get_adc2_values() {
  static int adc2_mux = 0x8;
  int value = ads2->readADC();
  ads2->setMultiplexer(adc2_mux);
  
  if (value == 0 || value == 0xff) {
    adc2_fail_count++;
    if (adc2_fail_count % ADC_FAIL_THRESHHOLD == 0) {
      Serial.println("ADC 2 failed: resetting");
      ads2->reset();
    }
    return;
  }

  adc2_fail_count = 0;

  uint64_t now = micros();

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
}

void record(uint64_t time, Channel channel, uint16_t value) {
  writer->write_uint16(channel_ids[channel], value, time);
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}