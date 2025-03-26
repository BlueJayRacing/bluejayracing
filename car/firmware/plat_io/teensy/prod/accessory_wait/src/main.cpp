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

// Thread modules
#include "adc/adc_thread.hpp"
#include "storage/sd_thread.hpp"
#include "network/pbudp_thread.hpp"      // Combined PB+UDP thread

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
    
    baja::util::Debug::info(F("Baja Data Acquisition System - Fast Path Version"));
    baja::util::Debug::info(F("Initializing..."));
    
    // Log memory usage at start
    uint32_t freeRam = getFreeRAM();
    freeRamLow = freeRam;
    baja::util::Debug::info(F("Initial free RAM: ") + String(freeRam) + F(" bytes"));
    
    // Set up the correct time
    setupTime();
    
    // Initialize SPI
    SPI.begin();
    
    // Set up thread timing
    threads.setSliceMicros(baja::config::THREAD_SLICE_MICROS);
    baja::util::Debug::info(F("Thread time slice set to ") + 
                          String(baja::config::THREAD_SLICE_MICROS) + F(" microseconds"));
    
    // Initialize channel configurations
    initializeChannelConfigs();

    // Initialize the SD card thread
    baja::util::Debug::info(F("Initializing SD card..."));
    // sdCardInitialized = baja::storage::SDThread::initialize(sampleBuffer, &sdRingBuf, SD_CS_PIN);
    
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
        baja::storage::SDThread::setChannelConfigs(channelConfigs);
        
        // Start SD writer thread
        baja::util::Debug::info(F("Starting SD writer thread..."));
        int sdThreadId = baja::storage::SDThread::start();
        
        if (sdThreadId < 0) {
            baja::util::Debug::error(F("Failed to start SD writer thread!"));
        } else {
            baja::util::Debug::info(F("SD writer thread started with ID: ") + String(sdThreadId) + 
                                  F(", priority: ") + String(baja::config::SD_WRITER_THREAD_PRIORITY));
        }
    } else {
        baja::util::Debug::error(F("SD card initialization failed!"));
    }
    
    // Initialize ADC with default settings
    baja::util::Debug::info(F("Initializing ADC..."));
    baja::adc::ADCSettings adcSettings;
    adcSettings.deviceType = ID_AD7175_8;
    adcSettings.referenceSource = INTERNAL_REF;
    adcSettings.operatingMode = CONTINUOUS;
    adcSettings.readStatusWithData = true;
    adcSettings.odrSetting = SPS_25000;
    
    // Initialize the ADC thread with both buffers
    adcInitialized = baja::adc::ADCThread::initialize(
        sampleBuffer,      // Main buffer for SD storage
        fastBuffer,        // Fast buffer for network transmission
        ADC_CS_PIN, 
        SPI, 
        adcSettings
    );
    // adcInitialized = baja::adc::ADCThread::initialize(sampleBuffer, ADC_CS_PIN, SPI, adcSettings);
    
    if (adcInitialized) {
        baja::util::Debug::info(F("ADC initialized successfully."));
        
        // Configure ADC channels
        baja::util::Debug::info(F("Configuring ADC channels..."));
        bool configSuccess = baja::adc::ADCThread::configureChannels(
            channelConfigsArray, 
            baja::adc::ADC_CHANNEL_COUNT
        );
        
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
                baja::util::Debug::info(F("ADC thread started with ID: ") + String(adcThreadId) + 
                                      F(", priority: ") + String(baja::config::ADC_THREAD_PRIORITY));
            }
        }
    } else {
        baja::util::Debug::error(F("ADC initialization failed!"));
    }
    
    // // Initialize the combined PB+UDP thread if ADC is working
    // if (adcInitialized) {
    //     // Initialize the combined PB+UDP thread with the fast buffer
    //     baja::util::Debug::info(F("Initializing combined PB+UDP thread..."));
    //     baja::util::Debug::info(F("Server: ") + String(UDP_SERVER_ADDRESS) + F(":") + String(UDP_SERVER_PORT));
        
    //     bool pbpdpInitialized = baja::network::PBUDPThread::initialize(
    //         fastBuffer,
    //         UDP_SERVER_ADDRESS,
    //         UDP_SERVER_PORT
    //     );
        
    //     if (pbpdpInitialized) {
    //         baja::util::Debug::info(F("Combined PB+UDP thread initialized."));
    //         networkInitialized = true;
            
    //         // Start the combined PB+UDP thread
    //         baja::util::Debug::info(F("Starting combined PB+UDP thread..."));
    //         int pbpdpThreadId = baja::network::PBUDPThread::start();
            
    //         if (pbpdpThreadId < 0) {
    //             baja::util::Debug::error(F("Failed to start combined PB+UDP thread!"));
    //             networkInitialized = false;
    //         } else {
    //             baja::util::Debug::info(F("Combined PB+UDP thread started with ID: ") + String(pbpdpThreadId) + 
    //                                   F(", priority: ") + String(baja::config::PBUDP_THREAD_PRIORITY));
    //         }
    //     } else {
    //         baja::util::Debug::error(F("Combined PB+UDP thread initialization failed!"));
    //         networkInitialized = false;
    //     }
    // }
    
    // Log configuration summary
    baja::util::Debug::info(F("\n========== CONFIGURATION SUMMARY =========="));
    baja::util::Debug::info(F("Main buffer size: ") + String(baja::config::SAMPLE_RING_BUFFER_SIZE) + F(" samples"));
    baja::util::Debug::info(F("Fast buffer size: ") + String(baja::config::FAST_BUFFER_SIZE) + F(" samples"));
    baja::util::Debug::info(F("Downsampling ratio: 1:") + String(baja::config::FAST_BUFFER_DOWNSAMPLE_RATIO));
    baja::util::Debug::info(F("Encoding mode: ") + String(baja::config::USE_VERBOSE_DATA_CHUNK ? "Verbose" : "Fixed"));
    baja::util::Debug::info(F("Hard-coded encoding: ") + String(baja::config::USE_HARD_CODED_ENCODING ? "Enabled" : "Disabled"));
    baja::util::Debug::info(F("Fixed samples per batch: ") + String(baja::config::FIXED_SAMPLE_COUNT));
    baja::util::Debug::info(F("Thread slice: ") + String(baja::config::THREAD_SLICE_MICROS) + F(" µs"));
    baja::util::Debug::info(F("==========================================\n"));
    
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
            baja::util::Debug::warning(F("SD Writer thread is not running! Attempting to restart..."));
            
            // Attempt to restart the thread
            int sdThreadId = baja::storage::SDThread::start();
            
            if (sdThreadId > 0) {
                baja::util::Debug::info(F("SD Writer thread restarted with ID: ") + String(sdThreadId));
            } else {
                baja::util::Debug::error(F("Failed to restart SD Writer thread!"));
            }
        }
        
        // Check ADC thread health
        if (adcInitialized && !baja::adc::ADCThread::isRunning()) {
            baja::util::Debug::warning(F("ADC thread is not running! Attempting to restart..."));
            
            // Attempt to restart the thread
            int adcThreadId = baja::adc::ADCThread::start();
            
            if (adcThreadId > 0) {
                baja::util::Debug::info(F("ADC thread restarted with ID: ") + String(adcThreadId));
            } else {
                baja::util::Debug::error(F("Failed to restart ADC thread!"));
            }
        }
        
        // Check combined PB+UDP thread health
        if (networkInitialized && !baja::network::PBUDPThread::isRunning()) {
            baja::util::Debug::warning(F("Combined PB+UDP thread is not running! Attempting to restart..."));
            
            // Attempt to restart the thread
            int pbpdpThreadId = baja::network::PBUDPThread::start();
            
            if (pbpdpThreadId > 0) {
                baja::util::Debug::info(F("Combined PB+UDP thread restarted with ID: ") + String(pbpdpThreadId));
            } else {
                baja::util::Debug::error(F("Failed to restart combined PB+UDP thread!"));
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
        float samplesPerSecond = sampleDelta / 30.0f;
        baja::util::Debug::info(F("New samples in last 30 seconds: ") + 
                           String(sampleDelta) + 
                           F(" (") + String(samplesPerSecond, 1) + F(" samples/sec)"));
        lastSampleCount = currentSampleCount;
        
        baja::util::Debug::info(F("\n========== SYSTEM STATUS =========="));
        baja::util::Debug::info(F("Uptime: ") + String((currentTime - startTime) / 1000) + F(" seconds"));
        baja::util::Debug::info(F("Samples collected: ") + String(currentSampleCount));
        
        if (adcInitialized && baja::adc::ADCThread::getHandler()) {
            baja::util::Debug::info(F("Active channel: ") + 
                String(baja::adc::ADCThread::getHandler()->getActiveChannel()));
        }
        
        // Print buffer statistics
        baja::util::Debug::info(F("Main buffer: ") + 
                         String(sampleBuffer.available()) + F("/") + 
                         String(sampleBuffer.capacity()) + F(" samples (") + 
                         String(100.0f * sampleBuffer.available() / sampleBuffer.capacity(), 1) + F("%)"));
        
        // if (baja::adc::ADCThread::getFastBuffer()) {
        //     baja::buffer::CircularBuffer<baja::data::ChannelSample, baja::config::FAST_BUFFER_SIZE>* fastBufferPtr = 
        //         baja::adc::ADCThread::getFastBuffer();
                
        //     baja::util::Debug::info(F("Fast buffer: ") + 
        //                      String(fastBufferPtr->available()) + F("/") + 
        //                      String(fastBufferPtr->capacity()) + F(" samples (") + 
        //                      String(100.0f * fastBufferPtr->available() / fastBufferPtr->capacity(), 1) + F("%), ") +
        //                      F("Overwrites: ") + String(fastBufferPtr->getOverwriteCount()));
        // }
        
        // Get combined PB+UDP thread stats
        if (networkInitialized && baja::network::PBUDPThread::getHandler()) {
            uint32_t messagesSent = 0, sampleCount = 0, bytesTransferred = 0, sendErrors = 0;
            baja::network::PBUDPThread::getStats(messagesSent, sampleCount, bytesTransferred, sendErrors);
            
            float samplesPerMsg = messagesSent > 0 ? (float)sampleCount / messagesSent : 0;
            float msgsPerSec = messagesSent / ((currentTime - startTime) / 1000.0f);
            float kbytesPerSec = bytesTransferred / ((currentTime - startTime) / 1000.0f) / 1024.0f;
            
            baja::util::Debug::info(F("Network stats: ") + String(messagesSent) + F(" messages sent (") + 
                                  String(msgsPerSec, 1) + F(" msgs/sec), ") + 
                                  String(sampleCount) + F(" samples processed (") +
                                  String(samplesPerMsg, 1) + F(" samples/msg), ") +
                                  String(bytesTransferred / 1024.0f, 1) + F(" KB transferred (") +
                                  String(kbytesPerSec, 1) + F(" KB/sec)"));
            
            if (sendErrors > 0) {
                baja::util::Debug::warning(F("Network send errors: ") + String(sendErrors));
            }
        }
        
        // // Log SD info
        // if (sdCardInitialized && baja::storage::SDThread::getWriter()) {
        //     baja::util::Debug::info(F("SD file: ") + 
        //         String(baja::storage::SDThread::getWriter()->getCurrentFilename().c_str()));
            
        //     uint32_t bytesWritten = baja::storage::SDThread::getWriter()->getBytesWritten();
        //     uint32_t samplesWritten = baja::storage::SDThread::getWriter()->getSamplesWritten();
        //     float bytesPerSample = samplesWritten > 0 ? (float)bytesWritten / samplesWritten : 0;
            
        //     baja::util::Debug::info(F("SD stats: ") + String(bytesWritten / 1024.0f, 1) + F(" KB written, ") + 
        //                           String(samplesWritten) + F(" samples (") + 
        //                           String(bytesPerSample, 1) + F(" bytes/sample)"));
        // }
        
        // Log memory usage
        uint32_t freeRam = getFreeRAM();
        baja::util::Debug::info(F("Memory: Free RAM = ") + String(freeRam) + F(" bytes, Low mark = ") + 
                             String(freeRamLow) + F(" bytes"));
        
        baja::util::Debug::info(F("===================================\n"));
    }
    
    // Check if we have no samples after 10 seconds
    static bool noSamplesWarningPrinted = false;
    if (adcInitialized && currentTime - startTime > 10000 && 
        baja::adc::ADCThread::getHandler() && 
        baja::adc::ADCThread::getHandler()->getSampleCount() == 0 && 
        !noSamplesWarningPrinted) {
        baja::util::Debug::warning(F("\n*** WARNING: No samples collected after 10 seconds! ***"));
        baja::util::Debug::warning(F("Check ADC communication and waitForReady() operation"));
        noSamplesWarningPrinted = true;
    }
    
    // Small yield for other tasks
    yield();
    resetWatchdog();
    
    // Monitor free RAM
    uint32_t freeRam = getFreeRAM();
    if (freeRam < freeRamLow) {
        freeRamLow = freeRam;
        
        // Only log significant changes to reduce spam
        static uint32_t lastRamLogTime = 0;
        if (currentTime - lastRamLogTime > 5000) {
            baja::util::Debug::info(F("Free RAM low mark update: ") + String(freeRamLow) + F(" bytes"));
            lastRamLogTime = currentTime;
        }
    }
    
    // Check for critically low memory
    if (freeRam < 10000) {
        static uint32_t lastLowMemoryWarningTime = 0;
        if (currentTime - lastLowMemoryWarningTime > 10000) {
            baja::util::Debug::warning(F("CRITICAL: Memory critically low (") + 
                                     String(freeRam) + F(" bytes), consider system reset"));
            lastLowMemoryWarningTime = currentTime;
        }
        
        // If extremely critical, force reset
        if (freeRam < 2000) {
            baja::util::Debug::error(F("FATAL: Memory exhausted, triggering watchdog reset"));
            delay(1000);
            // Intentionally do not reset watchdog to trigger reset
            while(1) { }
        }
    }
}