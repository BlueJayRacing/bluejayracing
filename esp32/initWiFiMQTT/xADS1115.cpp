#include "xADS1115.h"

#define SDA -1
#define SCL -1
#define WIRE_FREQ 400000

bool xADS1115::ads_ready = false;

xADS1115::xADS1115(int ads_intrupt_pin, int gain, int data_rate){
  ADS = ADS1115(0x48);
  this->ads_intrupt_pin = ads_intrupt_pin;
  this->gain = gain;
  this->data_rate = data_rate;
}

void xADS1115::beginADS(){
  Wire.begin(SDA, SCL, WIRE_FREQ);
  pinMode(ads_intrupt_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ads_intrupt_pin), xADS1115::makeADSReady, RISING);

  ADS.begin();
  ADS.setGain(gain);      //  6.144 volt
  ADS.setDataRate(data_rate);  //  0 = slow   4 = medium   7 = fast
  //  SET ALERT RDY PIN (QueConvert mode)
  //  set the MSB of the Hi_thresh register to 1
  ADS.setComparatorThresholdHigh(0x8000);
  //  set the MSB of the Lo_thresh register to 0
  ADS.setComparatorThresholdLow(0x0000);
  ADS.setComparatorQueConvert(0);
  ADS.setMode(0);      //  continuous mode
  ADS.requestADC_Differential_0_1();

  Serial.println("ADS0 modes set");
}

void xADS1115::makeADSReady(){
  ads_ready = true;
}

int16_t xADS1115::handleConversion(){
  if (ads_ready == false){
     return NULL;
  }
  ads_ready = false;
  return ADS.getValue();
}