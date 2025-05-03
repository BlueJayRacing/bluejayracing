#include <Arduino.h>
#include <TeensyThreads.h>
#include <SPI.h>
#include <TimeLib.h>
#include <QNEthernet.h>
#include <AsyncUDP_Teensy41.h> // need to include h once in main and hpp elsewhere where needed 

// Configuration
#include "config/config.hpp"
#include "config/defines.h"

// Utilities
#include "util/ring_buffer.hpp"
#include "util/circular_buffer.hpp"
#include "util/sample_data.hpp"
#include "util/debug_util.hpp"
#include "util/channel_mapping.hpp"

// Thread modules
#include "adc/adc_functions.hpp"
#include "storage/sd_functions.hpp"
#include "network/pbudp_functions.hpp"      // Combined PB+UDP thread
#include "digital/digital_functions.hpp"    // Digital input monitoring

// NEW: Time functions module (our NTP/SRTC updater)
#include "ntp/time_functions.hpp"

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

// Fast path buffer in DMAMEM for low-latency network transmission
DMAMEM baja::data::ChannelSample fastBufferStorage[baja::config::FAST_BUFFER_SIZE];

// Create all the ring buffers with external storage
baja::buffer::RingBuffer<baja::data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE> sampleBuffer(ringBufferStorage);
baja::buffer::CircularBuffer<baja::data::ChannelSample, baja::config::FAST_BUFFER_SIZE> fastBuffer(fastBufferStorage);

// Global status flags
bool adcInitialized = false;
bool sdCardInitialized = false;
bool networkInitialized = false;
bool digitalInitialized = false;    // Digital input status flag
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
    Serial.println(F("Current time: ") + String(timeStr));
}

// Print consolidated system status
void printSystemStatus() {
    uint32_t currentTime = millis();
    
    baja::util::Debug::info(F("\n========== SYSTEM STATUS =========="));
    baja::util::Debug::info(F("Uptime: ") + String((currentTime - startTime) / 1000) + F(" seconds"));
    
    // Print ADC status
    if (adcInitialized && baja::adc::functions::isRunning()) {
        uint64_t sampleCount = baja::adc::functions::getSampleCount();
        uint8_t activeChannel = baja::adc::functions::getActiveChannel();
        
        // Calculate samples per second
        static uint64_t lastSampleCount = 0;
        static uint32_t lastSampleTime = 0;
        float samplesPerSecond = 0;
        
        if (lastSampleTime > 0) {
            samplesPerSecond = (sampleCount - lastSampleCount) * 1000.0f / (currentTime - lastSampleTime);
        }
        
        lastSampleCount = sampleCount;
        lastSampleTime = currentTime;
        
        baja::util::Debug::info(F("ADC: Samples: ") + String(sampleCount) + 
                             F(", Rate: ") + String(samplesPerSecond, 1) + F(" sps") +
                             F(", Active ADC Ch: ") + String(activeChannel));
        
        // Print ADC timing stats
        float avgTime;
        uint32_t minTime, maxTime;
        uint64_t sampleCountTiming;
        baja::adc::functions::getTimingStats(avgTime, minTime, maxTime, sampleCountTiming);
        
        baja::util::Debug::info(F("ADC Timing: avg=") + String(avgTime, 1) + 
                             F("µs, min=") + String(minTime) + 
                             F("µs, max=") + String(maxTime) + F("µs"));
    }
    
    // Print digital input status
    if (digitalInitialized && baja::digital::functions::isRunning()) {
        uint64_t sampleCount = baja::digital::functions::getSampleCount();
        
        baja::util::Debug::info(F("Digital: Samples: ") + String(sampleCount));
        
        // Print digital timing stats
        float avgTime;
        uint32_t minTime, maxTime;
        uint64_t sampleCountTiming;
        baja::digital::functions::getTimingStats(avgTime, minTime, maxTime, sampleCountTiming);
        
        baja::util::Debug::info(F("Digital Timing: avg=") + String(avgTime, 1) + 
                            F("µs, min=") + String(minTime) + 
                            F("µs, max=") + String(maxTime) + F("µs"));
    }
    
    // Print buffer statistics
    if (adcInitialized) {
        baja::util::Debug::info(F("Main buffer: ") + 
                             String(sampleBuffer.available()) + F("/") + 
                             String(sampleBuffer.capacity()) + F(" samples (") + 
                             String(100.0f * sampleBuffer.available() / sampleBuffer.capacity(), 1) + F("%)"));
        
        baja::util::Debug::info(F("Fast buffer: ") + 
                             String(fastBuffer.available()) + F("/") + 
                             String(fastBuffer.capacity()) + F(" samples (") + 
                             String(100.0f * fastBuffer.available() / fastBuffer.capacity(), 1) + F("%), ") +
                             F("Overwrites: ") + String(fastBuffer.getOverwriteCount()));
    }
    
    // Print SD writer status
    if (sdCardInitialized && baja::storage::functions::isRunning()) {
        baja::util::Debug::info(F("SD file: ") + 
                             String(baja::storage::functions::getCurrentFilename().c_str()));
        
        uint32_t bytesWritten = baja::storage::functions::getBytesWritten();
        uint64_t samplesWritten = baja::storage::functions::getSamplesWritten();
        float bytesPerSample = samplesWritten > 0 ? (float)bytesWritten / samplesWritten : 0;
        
        baja::util::Debug::info(F("SD stats: ") + String(bytesWritten / 1024.0f, 1) + F(" KB written, ") + 
                             String(samplesWritten) + F(" samples (") + 
                             String(bytesPerSample, 1) + F(" bytes/sample)"));
        
        // Print SD timing stats
        float avgTime;
        uint32_t minTime, maxTime, totalWrites;
        baja::storage::functions::getTimingStats(avgTime, minTime, maxTime, totalWrites);
        
        baja::util::Debug::info(F("SD Timing: avg=") + String(avgTime, 1) + 
                             F("µs, min=") + String(minTime) + 
                             F("µs, max=") + String(maxTime) + 
                             F("µs, writes=") + String(totalWrites));
    }
    
    // Print network status
    if (networkInitialized && baja::network::functions::isRunning()) {
        uint32_t messagesSent = 0, sampleCount = 0, bytesTransferred = 0, sendErrors = 0;
        baja::network::functions::getStats(messagesSent, sampleCount, bytesTransferred, sendErrors);
        
        float samplesPerMsg = messagesSent > 0 ? (float)sampleCount / messagesSent : 0;
        float msgsPerSec = messagesSent / ((currentTime - startTime) / 1000.0f);
        float kbytesPerSec = bytesTransferred / ((currentTime - startTime) / 1000.0f) / 1024.0f;
        
        baja::util::Debug::info(F("Network stats: ") + String(messagesSent) + F(" messages sent (") + 
                             String(msgsPerSec, 1) + F(" msgs/sec), ") + 
                             String(sampleCount) + F(" samples (") +
                             String(samplesPerMsg, 1) + F(" samples/msg), ") +
                             String(bytesTransferred / 1024.0f, 1) + F(" KB (") +
                             String(kbytesPerSec, 1) + F(" KB/sec)"));
        
        // Print network timing stats
        float avgTime;
        uint32_t minTime, maxTime, messageCount;
        baja::network::functions::getTimingStats(avgTime, minTime, maxTime, messageCount);
        
        baja::util::Debug::info(F("Network Timing: avg=") + String(avgTime, 1) + 
                             F("µs, min=") + String(minTime) + 
                             F("µs, max=") + String(maxTime) + 
                             F("µs, msgs=") + String(messageCount));
        
        if (sendErrors > 0) {
            baja::util::Debug::warning(F("Network send errors: ") + String(sendErrors));
        }
    }
    
    // Log memory usage
    uint32_t freeRam = getFreeRAM();
    baja::util::Debug::info(F("Memory: Free RAM = ") + String(freeRam) + F(" bytes, Low mark = ") + 
                         String(freeRamLow) + F(" bytes"));
    
    baja::util::Debug::info(F("===================================\n"));
}


void setup() {
    // Initialize serial
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }
    
    // Record start time
    startTime = millis();
    
    // Initialize debug utility
    baja::util::Debug::init(baja::util::Debug::INFO);
    
    baja::util::Debug::info(F("Baja Data Acquisition System - Master Loop Version"));
    baja::util::Debug::info(F("Initializing..."));
    
    // Log memory usage at start
    uint32_t freeRam = getFreeRAM();
    freeRamLow = freeRam;
    baja::util::Debug::info(F("Initial free RAM: ") + String(freeRam) + F(" bytes"));
    
    // Set up the correct time
    setupTime();
    
    // Initialize SPI
    SPI.begin();
    
    // Initialize channel configurations
    bool configSuccess = baja::adc::initializeChannelConfigs(
        channelConfigsArray, 
        baja::config::ADC_ENABLE_ALL_CHANNELS
    );
    if (!configSuccess) {
        baja::util::Debug::error(F("Failed to initialize channel configurations!"));
    } else {
        // Print the channel configurations
        baja::adc::printChannelConfigs(channelConfigsArray);
    }

    // Initialize digital inputs
    baja::util::Debug::info(F("Initializing digital inputs..."));
    digitalInitialized = baja::digital::functions::initialize(
        sampleBuffer,
        fastBuffer
    );
    
    if (digitalInitialized) {
        baja::util::Debug::info(F("Digital inputs initialized successfully."));
        
        // Start digital monitoring
        baja::util::Debug::info(F("Starting digital input monitoring..."));
        if (!baja::digital::functions::start()) {
            baja::util::Debug::error(F("Failed to start digital input monitoring!"));
            digitalInitialized = false;
        } else {
            baja::util::Debug::info(F("Digital input monitoring started successfully"));
        }
    } else {
        baja::util::Debug::error(F("Digital input initialization failed!"));
    }

    // Initialize the SD card
    baja::util::Debug::info(F("Initializing SD card..."));
    sdCardInitialized = baja::storage::functions::initialize(
        sampleBuffer, 
        &sdRingBuf, 
        SD_CS_PIN
    );
    
    if (sdCardInitialized) {
        baja::util::Debug::info(F("SD card initialized successfully."));
        
        // Create vector of enabled channel configs
        std::vector<baja::adc::ChannelConfig> channelConfigs;
        for (int i = 0; i < baja::adc::ADC_CHANNEL_COUNT; i++) {
            if (channelConfigsArray[i].enabled) {
                channelConfigs.push_back(channelConfigsArray[i]);
            }
        }
        
        // Set channel names for SD writer
        baja::storage::functions::setChannelConfigs(channelConfigs);
        
        // Start SD writer
        baja::util::Debug::info(F("Starting SD writer..."));
        if (!baja::storage::functions::start()) {
            baja::util::Debug::error(F("Failed to start SD writer!"));
            sdCardInitialized = false;
        } else {
            baja::util::Debug::info(F("SD writer started successfully"));
        }
    } else {
        baja::util::Debug::error(F("SD card initialization failed!"));
    }
    
    // Initialize ADC with default settings
    baja::util::Debug::info(F("Initializing ADC..."));
    baja::adc::ADCSettings adcSettings;
    adcSettings.deviceType = ID_AD7175_8;
    adcSettings.referenceSource = EXTERNAL_REF2;
    adcSettings.operatingMode = CONTINUOUS;
    adcSettings.readStatusWithData = true;
    adcSettings.odrSetting = SPS_10000;
    
    // Initialize the ADC
    adcInitialized = baja::adc::functions::initialize(
        sampleBuffer,
        fastBuffer,
        ADC_CS_PIN, 
        SPI, 
        adcSettings
    );
    
    if (adcInitialized) {
        baja::util::Debug::info(F("ADC initialized successfully."));
        
        // Configure ADC channels
        baja::util::Debug::info(F("Configuring ADC channels..."));
        bool configSuccess = baja::adc::functions::configureChannels(
            channelConfigsArray, 
            baja::adc::ADC_CHANNEL_COUNT
        );
        
        if (!configSuccess) {
            baja::util::Debug::error(F("ADC channel configuration failed!"));
            adcInitialized = false;
        } else {
            baja::util::Debug::info(F("ADC channels configured successfully."));
            
            // Start ADC
            baja::util::Debug::info(F("Starting ADC..."));
            if (!baja::adc::functions::start()) {
                baja::util::Debug::error(F("Failed to start ADC!"));
                adcInitialized = false;
            } else {
                baja::util::Debug::info(F("ADC started successfully"));
            }
        }
    } else {
        baja::util::Debug::error(F("ADC initialization failed!"));
    }
    
    // Initialize the network if ADC is working
    if (adcInitialized) {
        baja::util::Debug::info(F("Initializing network..."));
        baja::util::Debug::info(F("Server: ") + String(UDP_SERVER_ADDRESS) + F(":") + String(UDP_SERVER_PORT));
        
        networkInitialized = baja::network::functions::initialize(
            fastBuffer,
            UDP_SERVER_ADDRESS,
            UDP_SERVER_PORT
        );
        
        if (networkInitialized) {
            baja::util::Debug::info(F("Network initialized successfully."));
            
            // Start network operations
            baja::util::Debug::info(F("Starting network operations..."));
            if (!baja::network::functions::start()) {
                baja::util::Debug::error(F("Failed to start network operations!"));
                networkInitialized = false;
            } else {
                baja::util::Debug::info(F("Network operations started successfully"));
            }
        } else {
            baja::util::Debug::error(F("Network initialization failed!"));
        }
    }

    baja::time::functions::initialize();
    
    // Log configuration summary
    baja::util::Debug::info(F("\n========== CONFIGURATION SUMMARY =========="));
    baja::util::Debug::info(F("Main buffer size: ") + String(baja::config::SAMPLE_RING_BUFFER_SIZE) + F(" samples"));
    baja::util::Debug::info(F("Fast buffer size: ") + String(baja::config::FAST_BUFFER_SIZE) + F(" samples"));
    baja::util::Debug::info(F("Downsampling ratio: 1:") + String(baja::config::FAST_BUFFER_DOWNSAMPLE_RATIO));
    baja::util::Debug::info(F("Hard-coded encoding: ") + String(baja::config::USE_HARD_CODED_ENCODING ? "Enabled" : "Disabled"));
    baja::util::Debug::info(F("Fixed samples per batch: ") + String(baja::config::FIXED_SAMPLE_COUNT));
    baja::util::Debug::info(F("Digital inputs: ") + String(digitalInitialized ? "Enabled" : "Disabled"));
    baja::util::Debug::info(F("==========================================\n"));
    
    baja::util::Debug::info(F("Initialization complete. System running."));
}


void loop() {
    // Increment loop counter
    loopCount++;
    
    // Always process ADC data - highest priority
    if (adcInitialized && baja::adc::functions::processSample()) {
        samplesProcessedTotal++;
    }
    
    // Process digital inputs every cycle
    if (digitalInitialized && baja::digital::functions::isRunning()) {
        baja::digital::functions::process();
    }
    
    // Process SD operations - only if enough samples are available
    // (This is already handled in SDWriter::process())
    if (sdCardInitialized && baja::storage::functions::isRunning() && loopCount % 5 == 0) {
        baja::storage::functions::process();
    }
    
    // Process network operations - only if enough samples are available
    // (This is already handled in PBUDPHandler::processAndSendBatch())
    if (networkInitialized && baja::network::functions::isRunning() && loopCount % 5 == 1) {
        baja::network::functions::process();
    }

    if (networkInitialized && loopCount % 100 == 0) {
        // Process NTP time updates
        baja::time::functions::update();
    }
    
    
    // System monitoring and status reporting (every 5 seconds)
    static uint32_t lastStatusTime = 0;
    uint32_t currentTime = millis();
    
    if (currentTime - lastStatusTime >= 15000) {
        lastStatusTime = currentTime;
        printSystemStatus();
    }
    
    // Service watchdog
    resetWatchdog();
}