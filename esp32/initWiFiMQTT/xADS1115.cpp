#include "xADS1115.h"

bool xADS1115::adsReady = false;
int xADS1115::count = 0;

xADS1115::xADS1115(int intruptPin, int gain, int dataRate){
  this->ADS_INTRUPT_PIN = intruptPin;
  this->gain = gain;
  this->dataRate = dataRate;
  ADS = ADS1115(0x48);
}

void xADS1115::beginADS(){
  Wire.begin(-1, -1, 400000);
  pinMode(ADS_INTRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ADS_INTRUPT_PIN), xADS1115::makeADSReady, RISING);

  ADS.begin();
  ADS.setGain(16);      //  6.144 volt
  ADS.setDataRate(7);  //  0 = slow   4 = medium   7 = fast
  //  SET ALERT RDY PIN (QueConvert mode)
  //  set the MSB of the Hi_thresh register to 1
  ADS.setComparatorThresholdHigh(0x8000);
  //  set the MSB of the Lo_thresh register to 0
  ADS.setComparatorThresholdLow(0x0000);
  ADS.setComparatorQueConvert(0);
  ADS.setMode(0);      //  continuous mode
  ADS.requestADC_Differential_0_1();

  Serial.println("ADS0 modes set");

  int value0 = ADS.readADC(0);      //  first read to trigger
  Serial.println(value0);
}

void xADS1115::makeADSReady(){
  adsReady = true;
  /*
  if (count % 10000 == 0){
    Serial.print("Time: ");
    Serial.println(millis());
  }
  */
}

int16_t xADS1115::handleConversion(){
  if (adsReady == false){
     return NULL;
  } else {
    adsReady = false;
    return ADS.getValue();
  }
}