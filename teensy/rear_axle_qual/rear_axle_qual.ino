//
//    FILE: ADS_continuous.ino
//  AUTHOR: Rob.Tillaart
// PURPOSE: read analog input
//     URL: https://github.com/RobTillaart/ADS1X15

//  test
//  connect 1 potmeter
//
//  GND ---[   x   ]------ 5V
//             |
//
//  measure at x (connect to AIN0).
//
//  See https://github.com/RobTillaart/ADS1X15/issues/49
//a

#include "ADS1X15.h"
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>

File file; //file
const int chipSelect = BUILTIN_SDCARD;



//  choose your sensor
ADS1115 ADS0(0x48);
ADS1115 ADS1(0x49);
ADS1115 ADS2(0x4A);
ADS1115 ADS3(0x4B);

int ADS_INT_0 = 2;
int ADS_INT_1 = 3;


uint16_t count = 0;

uint16_t value0 = 0;
uint16_t value1 = 0;
uint16_t value2 = 0;
uint16_t value3 = 0;

uint16_t prev  = 0;
uint32_t lastTime = 0;
uint32_t lastSample = 0;

volatile bool RDY0 = false;
volatile bool RDY1 = false;


const int ledPin = 13;

uint32_t last0 = micros();
uint32_t last1 = micros();

String s;

void setup()
{

  Serial.begin(115200);
  Serial.println("alive");
  
  while (!SD.begin(chipSelect)) {
    Serial.println("could not open sd");
    delay(10);
  }

  setSyncProvider(getTeensy3Time);
  
  s = String(hour())+ "_" + String(minute())+"_rear_axle.txt";
  file = SD.open(s.c_str(), FILE_WRITE);


  

  Wire.begin();

  pinMode(ADS_INT_0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ADS_INT_0), adsReady0, RISING);

  pinMode(ADS_INT_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ADS_INT_1), adsReady1, RISING);

  ADS0.begin();
  ADS0.setGain(8);      //  6.144 volt
  ADS0.setDataRate(2);  //  0 = slow   4 = medium   7 = fast
  
  //  SET ALERT RDY PIN (QueConvert mode)
  //  set the MSB of the Hi_thresh register to 1
  ADS0.setComparatorThresholdHigh(0x8000);
  //  set the MSB of the Lo_thresh register to 0
  ADS0.setComparatorThresholdLow(0x0000);
  ADS0.setComparatorQueConvert(0);
  ADS0.setMode(0);      //  continuous mode
  ADS0.requestADC_Differential_0_1();
//  ADS0.readADC(0);      //  first read to trigger

  ADS1.begin();
  ADS1.setGain(8);      //  6.144 volt
  ADS1.setDataRate(2);  //  0 = slow   4 = medium   7 = fast
  //  SET ALERT RDY PIN (QueConvert mode)
  //  set the MSB of the Hi_thresh register to 1
  ADS1.setComparatorThresholdHigh(0x8000);
  //  set the MSB of the Lo_thresh register to 0
  ADS1.setComparatorThresholdLow(0x0000);
  ADS1.setComparatorQueConvert(0);
  ADS1.setMode(0);      //  continuous mode
  ADS1.requestADC_Differential_0_1();
//  ADS1.readADC(0);      //  first read to trigger

  ADS2.begin();
  ADS2.setGain(0);      
  ADS2.setDataRate(2); 
  ADS2.setMode(0); 
  ADS2.requestADC(0);

  ADS3.begin();
  ADS3.setGain(0);      
  ADS3.setDataRate(2); 
  ADS3.setMode(0); 
  ADS3.requestADC(0);
//  ADS3.requestADC_Differential_0_1();
}


void loop() {
  

  if (handleConversion0() == true) {
    uint32_t now0 = micros();
    
    last0 = now0;

    file.print(now0);
    file.print(",");
    file.print(ADS0.toVoltage(value0),9);
    file.print(",");
    file.print(ADS1.toVoltage(value1),9);
    file.print(",");
    file.print(ADS1.toVoltage(value1),9);
    file.print(",");
    file.println(ADS3.toVoltage(value3),9);
    Serial.print(now0 - last0);
    Serial.print(",\tV0:");
    Serial.print(ADS0.toVoltage(value0),9);
    Serial.print(",\tV1:"); 
    Serial.print(ADS1.toVoltage(value1),9);
    Serial.print(",\tV2:"); 
    Serial.println(ADS3.toVoltage(value3),9);
    Serial.print(",\tV3:"); 
    Serial.println(ADS3.toVoltage(value3),9);
    count++;
  }
  
  if (handleConversion1() == true) {
    uint32_t now1 = micros();
    
    last1 = now1;
    
//    Serial.print(now1 - last1);
//    Serial.print("\t0:");
//    Serial.print(ADS1.toVoltage(value1),5);
//    Serial.println();
    count++;
  }
  if (count > 5000) {
    file.close();
    file = SD.open(s.c_str(), FILE_WRITE);
  }
}

void adsReady0()
{
  RDY0 = true;
}

void adsReady1()
{
  RDY1 = true;
}

bool handleConversion0()
{
  if (RDY0 == false) return false;
  value0 = ADS0.getValue();
  value2 = ADS3.getValue();
  value3 = ADS3.getValue();
  RDY0 = false;
  return true;
}

bool handleConversion1()
{
  if (RDY1 == false) return false;
  value1 = ADS1.getValue();
  
  RDY1 = false;
  return true;
}


time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}


//  -- END OF FILE --
