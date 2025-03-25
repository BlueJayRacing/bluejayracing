#include <Arduino.h>
#include <TeensyThreads.h>
#include <SPI.h>
#include <TimeLib.h>

// Configuration
#include "config/config.hpp"
#include "config/defines.h"

// Utilities
#include "util/ring_buffer.hpp"
#include "util/sample_data.hpp"
#include "util/debug_util.hpp"

// Thread modules
#include "adc/adc_thread.hpp"
// Replace HTTP with UDP
// #include "network/http_thread.hpp"
#include "network/udp_thread.hpp"
#include "storage/sd_thread.hpp"
#include "serialization/pb_thread.hpp"
#include <AsyncUDP_Teensy41.h>

uint32_t freeRamLow = UINT32_MAX;

// Pin definitions
const uint8_t ADC_CS_PIN = 10;     // ADC chip select pin
const uint8_t SD_CS_PIN = 254;     // SD card CS pin (for SPI fallback)

// UDP server configuration
const char* UDP_SERVER_ADDRESS = "192.168.20.3";
const uint16_t UDP_SERVER_PORT = 8888;

// Create global buffers in RAM2/EXTMEM to reduce RAM1 usage
EXTMEM baja::data::ChannelSample ringBufferStorage[baja::config::SAMPLE_RING_BUFFER_SIZE];
EXTMEM baja::adc::ChannelConfig channelConfigsArray[baja::adc::ADC_CHANNEL_COUNT];

// Define the SdFat RingBuf in EXTMEM
DMAMEM RingBuf<FsFile, baja::config::SD_RING_BUF_CAPACITY> sdRingBuf;

// Create the two new buffers for protobuf serialization
EXTMEM baja::serialization::EncodedMessage encodedBufferStorage[baja::config::PB_MESSAGE_BUFFER_SIZE];

// Create all the ring buffers with external storage
baja::buffer::RingBuffer<baja::data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE> sampleBuffer(ringBufferStorage);
baja::buffer::RingBuffer<baja::serialization::EncodedMessage, baja::config::PB_MESSAGE_BUFFER_SIZE> encodedBuffer(encodedBufferStorage);

// Global status flags
bool adcInitialized = false;
bool sdCardInitialized = false;
uint32_t loopCount = 0;
uint32_t samplesProcessedTotal = 0;
uint32_t startTime = 0;
uint32_t lastSampleCount = 0;

// Function to get time from Teensy RTC
time_t getTeensyTime() {
    return Teensy3Clock.get();
}

void resetWatchdog() {
    // Service the watchdog by writing the WDOG service sequence
    WDOG1_WSR = 0x5555;
    WDOG1_WSR = 0xAAAA;
  }

void dateTime(uint16_t *date, uint16_t *time) {
    *date = FAT_DATE(year(), month(), day());
    *time = FAT_TIME(hour(), minute(), second());
}

// Function to get free RAM on Teensy 4.1
uint32_t getFreeRAM() {
    extern unsigned long _heap_start;
    extern unsigned long _heap_end;
    extern char *__brkval;
    
    uint32_t free_memory;
    
    if (__brkval == 0)
      free_memory = ((uint32_t)&_heap_end - (uint32_t)&_heap_start);
    else
      free_memory = ((uint32_t)&_heap_end - (uint32_t)__brkval);
      
    return free_memory;
}
  

// Function to set up the correct time for the Teensy
void setupTime() {
    setSyncProvider(getTeensyTime);  
    SdFile::dateTimeCallback(dateTime);

    // Set a default time if no time is set
    if (year() < 2023) {
        setTime(0, 0, 0, 1, 1, 2024);
        baja::util::Debug::info(F("Setting default time: 2024-01-01 00:00:00"));
    }
    
    // Print current time
    char timeStr[32];
    sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
            year(), month(), day(), hour(), minute(), second());
    baja::util::Debug::info(F("Current time: ") + String(timeStr));
}

// Initialize channel configurations
void initializeChannelConfigs() {
    baja::util::Debug::info(F("Initializing channel configurations"));
    
    for (int i = 0; i < baja::adc::ADC_CHANNEL_COUNT; i++) {
        channelConfigsArray[i].channelIndex = i;
        channelConfigsArray[i].analogInputs.ainp.pos_input = static_cast<ad717x_analog_input_t>(i);
        channelConfigsArray[i].analogInputs.ainp.neg_input = REF_M;
        channelConfigsArray[i].gain = baja::config::ADC_DEFAULT_GAIN;
        channelConfigsArray[i].setupIndex = 0;
        channelConfigsArray[i].enabled = baja::config::ADC_ENABLE_ALL_CHANNELS ? true : (i < 2);
        
        char name[16];
        snprintf(name, sizeof(name), "Channel_%d", i);
        channelConfigsArray[i].name = name;
    }
}

void setup() {
    // Initialize serial
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }
    
    // Record start time
    startTime = millis();
    
    // Initialize debug utility
    baja::util::Debug::init(baja::util::Debug::INFO);
    
    baja::util::Debug::info(F("Baja Data Acquisition System"));
    baja::util::Debug::info(F("Initializing..."));
    
    // Set up the correct time
    setupTime();
    
    // Initialize SPI
    SPI.begin();
    
    // Set up thread timing
    threads.setSliceMicros(baja::config::THREAD_SLICE_MICROS);
    
    // Initialize channel configurations
    initializeChannelConfigs();

    // Initialize the SD card thread
    sdCardInitialized = baja::storage::SDThread::initialize(sampleBuffer, &sdRingBuf, SD_CS_PIN);
    
    if (sdCardInitialized) {
        baja::util::Debug::info(F("SD card initialized."));
        
        // Create vector of enabled channel configs
        std::vector<baja::adc::ChannelConfig> channelConfigs;
        for (int i = 0; i < baja::adc::ADC_CHANNEL_COUNT; i++) {
            if (channelConfigsArray[i].enabled) {
                channelConfigs.push_back(channelConfigsArray[i]);
            }
        }
        
        // Set channel names for SD writer
        baja::storage::SDThread::setChannelConfigs(channelConfigs);
        
        // Start SD writer thread
        baja::util::Debug::info(F("Starting SD writer thread..."));
        int sdThreadId = baja::storage::SDThread::start();
        
        if (sdThreadId < 0) {
            baja::util::Debug::error(F("Failed to start SD writer thread!"));
        } else {
            baja::util::Debug::info(F("SD writer thread started with ID: ") + String(sdThreadId));
        }
    } else {
        baja::util::Debug::error(F("SD card initialization failed!"));
    }
    
    // Initialize ADC with default settings
    baja::adc::ADCSettings adcSettings;
    adcSettings.deviceType = ID_AD7175_8;
    adcSettings.referenceSource = INTERNAL_REF;
    adcSettings.operatingMode = CONTINUOUS;
    adcSettings.readStatusWithData = true;
    adcSettings.odrSetting = SPS_2500;
    
    // Initialize the ADC thread
    adcInitialized = baja::adc::ADCThread::initialize(sampleBuffer, ADC_CS_PIN, SPI, adcSettings);
    
    if (adcInitialized) {
        baja::util::Debug::info(F("ADC initialized successfully."));
        
        // Configure ADC channels
        bool configSuccess = baja::adc::ADCThread::configureChannels(channelConfigsArray, baja::adc::ADC_CHANNEL_COUNT);
        
        if (!configSuccess) {
            baja::util::Debug::error(F("ADC channel configuration failed!"));
            adcInitialized = false;
        } else {
            baja::util::Debug::info(F("ADC channels configured successfully."));
            
            // Start ADC thread
            baja::util::Debug::info(F("Starting ADC thread..."));
            int adcThreadId = baja::adc::ADCThread::start();
            
            if (adcThreadId < 0) {
                baja::util::Debug::error(F("Failed to start ADC thread!"));
                adcInitialized = false;
            } else {
                baja::util::Debug::info(F("ADC thread started with ID: ") + String(adcThreadId));
            }
        }
    } else {
        baja::util::Debug::error(F("ADC initialization failed!"));
    }
    
    // Initialize PB serialization thread if ADC is working
    if (adcInitialized) {
        // Initialize PB serialization thread
        bool pbInitialized = baja::serialization::PBThread::initialize(
            sampleBuffer, encodedBuffer);
        
        if (pbInitialized) {
            baja::util::Debug::info(F("PB serialization thread initialized."));
            
            // Start PB serialization thread
            baja::util::Debug::info(F("Starting PB serialization thread..."));
            int pbThreadId = baja::serialization::PBThread::start();
            
            if (pbThreadId < 0) {
                baja::util::Debug::error(F("Failed to start PB serialization thread!"));
            } else {
                baja::util::Debug::info(F("PB serialization thread started with ID: ") + String(pbThreadId));
            }
            
            // Initialize UDP client thread with encoded buffer
            bool udpInitialized = baja::network::UDPThread::initialize(
                encodedBuffer, 
                UDP_SERVER_ADDRESS,
                UDP_SERVER_PORT
            );
            
            if (udpInitialized) {
                baja::util::Debug::info(F("UDP client initialized."));
                
                // Start UDP client thread
                baja::util::Debug::info(F("Starting UDP client thread..."));
                int udpThreadId = baja::network::UDPThread::start();
                
                if (udpThreadId < 0) {
                    baja::util::Debug::error(F("Failed to start UDP client thread!"));
                } else {
                    baja::util::Debug::info(F("UDP client thread started with ID: ") + String(udpThreadId));
                }
            } else {
                baja::util::Debug::error(F("UDP client initialization failed!"));
            }
        } else {
            baja::util::Debug::error(F("PB serialization thread initialization failed!"));
        }
    }
    
    baja::util::Debug::info(F("Initialization complete. System running."));
}

void loop() {
    // Increment loop counter
    loopCount++;
    
    // Process ADC data directly from main loop if needed
    if (adcInitialized && baja::adc::ADCThread::processSample()) {
        samplesProcessedTotal++;
        
        // Print visual progress indicator
        if (samplesProcessedTotal % 10000 == 0) {
            Serial.print(".");
            
            if (samplesProcessedTotal % 500000 == 0) {
                Serial.println();
            }
        }
    }
    
    // Monitor thread health periodically
    static uint32_t lastThreadCheckTime = 0;
    uint32_t currentTime = millis();
    
    // Check threads every 10 seconds
    if (currentTime - lastThreadCheckTime > 10000) {
        lastThreadCheckTime = currentTime;
        
        // Check SD writer thread health
        if (sdCardInitialized && !baja::storage::SDThread::isRunning()) {
            baja::util::Debug::warning("SD Writer thread is not running! Attempting to restart...");
            
            // Attempt to restart the thread
            int sdThreadId = baja::storage::SDThread::start();
            
            if (sdThreadId > 0) {
                baja::util::Debug::info("SD Writer thread restarted with ID: " + String(sdThreadId));
            } else {
                baja::util::Debug::error("Failed to restart SD Writer thread!");
            }
        }
        
        // Check ADC thread health
        if (adcInitialized && !baja::adc::ADCThread::isRunning()) {
            baja::util::Debug::warning("ADC thread is not running! Attempting to restart...");
            
            // Attempt to restart the thread
            int adcThreadId = baja::adc::ADCThread::start();
            
            if (adcThreadId > 0) {
                baja::util::Debug::info("ADC thread restarted with ID: " + String(adcThreadId));
            } else {
                baja::util::Debug::error("Failed to restart ADC thread!");
            }
        }
        
        // Check PB serialization thread health
        if (adcInitialized && !baja::serialization::PBThread::isRunning()) {
            baja::util::Debug::warning("PB serialization thread is not running! Attempting to restart...");
            
            // Attempt to restart the thread
            int pbThreadId = baja::serialization::PBThread::start();
            
            if (pbThreadId > 0) {
                baja::util::Debug::info("PB serialization thread restarted with ID: " + String(pbThreadId));
            } else {
                baja::util::Debug::error("Failed to restart PB serialization thread!");
            }
        }
        
        // Check UDP thread health
        if (adcInitialized && !baja::network::UDPThread::isRunning()) {
            baja::util::Debug::warning("UDP thread is not running! Attempting to restart...");
            
            // Attempt to restart the thread
            int udpThreadId = baja::network::UDPThread::start();
            
            if (udpThreadId > 0) {
                baja::util::Debug::info("UDP thread restarted with ID: " + String(udpThreadId));
            } else {
                baja::util::Debug::error("Failed to restart UDP thread!");
            }
        }
    }
    
    // Print status occasionally
    static uint32_t lastStatusTime = 0;
    
    // Print detailed status every 30 seconds
    if (currentTime - lastStatusTime > 30000) {
        lastStatusTime = currentTime;
        
        // Check for new samples
        uint32_t currentSampleCount = 0;
        if (adcInitialized && baja::adc::ADCThread::getHandler()) {
            currentSampleCount = baja::adc::ADCThread::getHandler()->getSampleCount();
        }
        
        uint32_t sampleDelta = currentSampleCount - lastSampleCount;
        baja::util::Debug::info("New samples in last 30 seconds: " + 
                           String(sampleDelta) + 
                           " (" + String(sampleDelta / 30.0f) + " samples/sec)");
        lastSampleCount = currentSampleCount;
        
        // Focus on protobuf logging - comment out other buffer stats
        // sampleBuffer.printStats();
        
        baja::util::Debug::info("\n========== SYSTEM STATUS ==========");
        baja::util::Debug::info("Uptime: " + String((currentTime - startTime) / 1000) + " seconds");
        // baja::util::Debug::info("Loop count: " + String(loopCount));
        baja::util::Debug::info("Samples collected: " + String(currentSampleCount));
        
        if (adcInitialized && baja::adc::ADCThread::getHandler()) {
            baja::util::Debug::info("Active channel: " + 
                String(baja::adc::ADCThread::getHandler()->getActiveChannel()));
        }
        
        // Focus on the essential buffer information
        baja::util::Debug::info("Sample buffer: " + 
                         String(sampleBuffer.available()) + "/" + 
                         String(sampleBuffer.capacity()) + " samples");
        
        baja::util::Debug::info("Encoded buffer: " + 
                         String(encodedBuffer.available()) + "/" + 
                         String(encodedBuffer.capacity()) + " messages");
        
        // Get PB serialization stats
        uint32_t encodedCount = 0, sampleCount = 0;
        baja::serialization::PBThread::getStats(encodedCount, sampleCount);
        baja::util::Debug::info("PB serializer: " + String(encodedCount) + " messages encoded, " + 
                                String(sampleCount) + " samples processed");
        
        // Get UDP stats
        if (baja::network::UDPThread::getClient()) {
            uint32_t messagesSent = 0, oversizedMessages = 0, bytesTransferred = 0, sendErrors = 0;
            baja::network::UDPThread::getStats(messagesSent, oversizedMessages, bytesTransferred, sendErrors);
            
            baja::util::Debug::info("UDP client: " + String(messagesSent) + " messages sent, " + 
                                   String(bytesTransferred / 1024.0f, 1) + " KB transferred");
            
            if (oversizedMessages > 0) {
                baja::util::Debug::warning("UDP client: " + String(oversizedMessages) + " oversized messages dropped");
            }
        }
        
        // Only log SD info if PB_DEBUG_LOGGING is false to reduce output
        if (!baja::config::PB_DEBUG_LOGGING && sdCardInitialized && baja::storage::SDThread::getWriter()) {
            baja::util::Debug::info("SD file: " + 
                String(baja::storage::SDThread::getWriter()->getCurrentFilename().c_str()));
        }
        
        baja::util::Debug::info("===================================\n");
    }
    
    // Check if we have no samples after 10 seconds
    static bool noSamplesWarningPrinted = false;
    if (adcInitialized && currentTime - startTime > 10000 && 
        baja::adc::ADCThread::getHandler() && 
        baja::adc::ADCThread::getHandler()->getSampleCount() == 0 && 
        !noSamplesWarningPrinted) {
        baja::util::Debug::warning("\n*** WARNING: No samples collected after 10 seconds! ***");
        baja::util::Debug::warning("Check ADC communication and waitForReady() operation");
        noSamplesWarningPrinted = true;
    }
    
    // Small yield for other tasks
    yield();
    resetWatchdog();
    // Ethernet will handle itself through the UDP library

    uint32_t freeRam = getFreeRAM();
  if (freeRam < freeRamLow) {
    freeRamLow = freeRam;
    
    Serial.print("Free RAM low mark: ");
    Serial.println(freeRamLow);
  }
  
  // Check for critically low memory
  if (freeRam < 10000) { // Adjust this threshold based on your application
    Serial.println("WARNING: Memory critically low, resetting all connections");
    delay(1000); // Give system time to recover
  }
}