#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>
#include "avr/pgmspace.h"
#include "ADS1X15.h"


// Allocate space for the buffer
const int bufferSize = 512;
unsigned long buffer[bufferSize];
int bufferIndex = 0;

const int chipSelect = BUILTIN_SDCARD;

// File related variables
File file;
char folder_name[40];  // Use a char array instead of String
String file_name = "run";
int current_test = 1;

// ADC related variables
ADS1115 ADS[4];
uint16_t val[16];
int idx = 0;

uint32_t last = 0, now = 0;


void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Ensure SD card can be opened, retry until successful
    while (!SD.begin(chipSelect)) {
        Serial.println("could not open SD card, reattempting...");
        delay(100);
    }

    // Create a new folder with the name as the current time
    sprintf(folder_name, "rear_axel_%04d-%02d-%02d_%02d.%02d.%02d", year(), month(), day(), hour(), minute(), second());

    // Wait for serial port to connect
    while (!Serial) {
        delay(10);
    }

    if (!SD.mkdir(folder_name)) {
        Serial.println("error creating folder");
    }
    
    Serial.println("started ADC setup...");
    Serial.println(__FILE__);
    Serial.print("ADS1X15_LIB_VERSION: ");
    Serial.println(ADS1X15_LIB_VERSION);

    Wire.begin();

    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t address = 0x48 + i;
        ADS[i] = ADS1115(address);

        Serial.print(address, HEX);
        Serial.print("  ");
        Serial.println(ADS[i].begin() ? "connected" : "not connected");

        //  0 = slow   4 = medium   7 = fast, but more noise
        ADS[i].setDataRate(4);
    }
    ADS_request_all();

    Serial.println("created folder, ready to record (closing serial)");
    Serial.end(); // Close serial port
}

void loop() {
    setSyncProvider(getTeensy3Time);

    // Index through buffer
    buffer[bufferIndex++] = micros();

    // Write to buffer if full
    if (bufferIndex >= bufferSize) {
        writeToSDCard();
    }


}

time_t getTeensy3Time() {
    return Teensy3Clock.get();
}

void writeToSDCard() {
    // Open file to append data
    String file_dir = "/" + String(folder_name) + "/" + file_name + String(current_test) + ".txt";
    file = SD.open(file_dir.c_str(), FILE_WRITE);
    if (file) {
        // Write buffer data to file
        for (int i = 0; i < bufferSize; i++) {
            file.println(buffer[i]);
        }
        file.close(); // Close the file
        bufferIndex = 0; // Reset buffer index
    }
    else {
        Serial.println("Error opening file");
    }
}



// ADC premade function
bool ADS_read_all()
{
    for (int i = 0; i < 4; i++)
    {
        if (ADS[i].isConnected() && ADS[i].isBusy()) return true;
    }
    //  Serial.print("IDX:\t");
    //  Serial.println(idx);
    for (int i = 0; i < 4; i++)
    {
        if (ADS[i].isConnected())
        {
            val[i * 4 + idx] = ADS[i].getValue();
        }
    }
    idx++;
    if (idx < 4)
    {
        ADS_request_all();
        return true;
    }
    idx = 0;
    return false;
}