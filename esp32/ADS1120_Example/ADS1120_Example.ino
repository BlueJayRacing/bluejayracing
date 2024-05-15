/*
  This example shows how to get continous conversions out of the ADS1120.
  Lucas Etchezuri 30/06/2020
*/
#define CS_PIN 5
#define DATA_READY_PIN 4

#include "xADS1120.hpp"

xADS1120* ads;
int count = 0;
int led = D10;

void setup()
{
  Serial.begin(115200);
	ads = new xADS1120();
  ads->begin(CS_PIN, DATA_READY_PIN);
  pinMode(led, OUTPUT);
  Serial.println("Starting");
}

void loop()
{
	int val = ads->readADC();
  

  if (val != NULL) {
    count++;
    Serial.println(val);
    /*
    if (count % 1000 == 0) {
      Serial.println(millis());
    }*/
  }
  delay(40);
}

