#include <Arduino.h>
#include <TeensyThreads.h>
#include <SPI.h>
#include <TimeLib.h>

#include "ring_buffer.hpp"
#include "sample_data.hpp"
#include "adc_handler.hpp"
#include "sd_writer.hpp"
#include "config.hpp"
#include "debug_util.hpp"



#include "async_http_client.hpp"
#include "defines.h"
#include "AsyncHTTPRequest_Teensy41.h"

// Pin definitions
const uint8_t ADC_CS_PIN = 10;     // ADC chip select pin
const uint8_t SD_CS_PIN = 254;     // SD card CS pin (for SPI fallback)

// Create global buffers in RAM2 to reduce RAM1 usage
EXTMEM baja::data::ChannelSample ringBufferStorage[baja::config::SAMPLE_RING_BUFFER_SIZE];
EXTMEM baja::adc::ChannelConfig channelConfigsArray[baja::adc::ADC_CHANNEL_COUNT];

// Define the SdFat RingBuf in EXTMEM
DMAMEM RingBuf<FsFile, baja::config::SD_RING_BUF_CAPACITY> sdRingBuf;

// Create the global ring buffer with external buffer
baja::buffer::RingBuffer<baja::data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE> sampleBuffer(ringBufferStorage);

// Create the ADC handler
baja::adc::ADC7175Handler adcHandler(sampleBuffer);

// Create the SD writer with external RingBuf
baja::storage::SDWriter sdWriter(sampleBuffer, &sdRingBuf);


// Create the HTTP client
baja::network::AsyncHTTPClient httpClient(sampleBuffer);

// Global status flags
bool adcInitialized = false;
bool sdCardInitialized = false;
uint32_t loopCount = 0;
uint32_t samplesProcessedTotal = 0;
uint32_t startTime = 0;
uint32_t lastSampleCount = 0;


// Flag for SD writer thread
volatile bool sdWriterRunning = false;

// Thread handle for SD writer thread
int sdWriterThreadId = -1;

// Function to get time from Teensy RTC
time_t getTeensyTime() {
    return Teensy3Clock.get();
}

void dateTime(uint16_t *date, uint16_t *time) {
    *date = FAT_DATE(year(), month(), day());
    *time = FAT_TIME(hour(), minute(), second());
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



using namespace qindesign::network;

// Create a global AsyncHTTPRequest instance
AsyncHTTPRequest httpRequest;

#define NOT_SEND_HEADER_AFTER_CONNECTED        true
#define ASYNC_HTTP_DEBUG_PORT     Serial
#define _ASYNC_HTTP_LOGLEVEL_     4

const int HTTP_REQUEST_INTERVAL = 3000; // 3 seconds between requests

// Your server URL
const char* HTTP_SERVER_URL = "http://192.168.20.3:9365/";



void httpClientThreadFunc() {
    baja::util::Debug::info("HTTP client thread started");
    
    // Wait for things to stabilize before sending first request
    delay(3000);
    
    uint32_t lastProcessTime = 0;
    uint32_t lastStatsTime = 0;
    uint32_t successCount = 0;
    uint32_t errorCount = 0;
    
    // Set channel configurations for the HTTP client
    httpClient.setChannelConfigs(channelConfigsArray);
    
    // Set downsample ratio (send fewer samples)
    httpClient.setDownsampleRatio(5);  // Only send 1 in every 5 samples
    
    while (true) {
        uint32_t currentTime = millis();
        
        // Process data every 100ms
        if (currentTime - lastProcessTime >= 100) {
            lastProcessTime = currentTime;
            
            // Try to process samples from the buffer
            size_t processed = httpClient.process();
            
            if (processed > 0) {
                successCount++;
            }
        }
        
        // Print stats every 30 seconds
        if (currentTime - lastStatsTime >= 30000) {
            lastStatsTime = currentTime;
            
            baja::util::Debug::info("HTTP client stats - Successful batches: " + 
                String(successCount) + ", Errors: " + String(errorCount));
                
            // Check network status
            bool connected = httpClient.isConnected();
            baja::util::Debug::info("Network status: " + String(connected ? "Connected" : "Disconnected"));
            
            // If not connected, try to reconnect
            if (!connected) {
                baja::util::Debug::info("Attempting to reconnect...");
                if (httpClient.reconnect()) {
                    baja::util::Debug::info("Reconnection successful");
                } else {
                    baja::util::Debug::error("Reconnection failed");
                    errorCount++;
                }
            }
        }
        
        // Small yield to allow other threads to run
        threads.yield();
        
        // Add a small delay to prevent tight loop
        delay(10);
    }
}


// SD writer thread function
void sdWriterThreadFunc() {
    baja::util::Debug::info(F("SD Writer thread started"));
    sdWriterRunning = true;
    
    // Set thread priority
    threads.setTimeSlice(threads.id(), baja::config::SD_WRITER_THREAD_PRIORITY);
    
    // Create a new data file
    if (!sdWriter.createNewFile()) {
        baja::util::Debug::error(F("SD Writer: Failed to create initial data file!"));
    } else {
        baja::util::Debug::info(F("SD Writer: Created initial file: ") + String(sdWriter.getCurrentFilename().c_str()));
    }
    
    // Main SD writer loop
    uint32_t totalWritten = 0;
    uint32_t lastStatusTime = 0;
    
    while (sdWriterRunning) {
        // Process samples and write to SD card
        size_t samplesWritten = sdWriter.process();
        totalWritten += samplesWritten;
        
        // Print status occasionally
        uint32_t now = millis();
        if (now - lastStatusTime > 30000) {
            baja::util::Debug::info(F("SD Writer: Samples written: ") + 
                                 String(totalWritten) + 
                                 F(", File: ") + 
                                 String(sdWriter.getCurrentFilename().c_str()) + 
                                 F(", Bytes: ") + 
                                 String(sdWriter.getBytesWritten()));
            lastStatusTime = now;
        }
        
        // Check health and attempt recovery if needed
        if (!sdWriter.isHealthy()) {
            baja::util::Debug::warning(F("SD Writer: Detected unhealthy state, attempting recovery"));
            sdWriter.closeFile();
            delay(100);
            
            if (sdWriter.createNewFile()) {
                baja::util::Debug::info(F("SD Writer: Recovery successful"));
                sdWriter.resetHealth();
            } else {
                baja::util::Debug::warning(F("SD Writer: Recovery failed, will retry later"));
                delay(5000);
            }
        }
        
        // Small yield for other threads
        threads.yield();
    }
    
    // Close the file before exiting
    sdWriter.closeFile();
    baja::util::Debug::info(F("SD Writer thread stopped"));
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

    // Initialize the SD card
    baja::util::Debug::info(F("Initializing SD card..."));
    if (!(sdCardInitialized = sdWriter.begin())) {
        baja::util::Debug::error(F("SD card initialization failed!"));
    } else {
        baja::util::Debug::info(F("SD card initialized."));
        
        // Create vector of enabled channel configs
        std::vector<baja::adc::ChannelConfig> channelConfigs;
        for (int i = 0; i < baja::adc::ADC_CHANNEL_COUNT; i++) {
            if (channelConfigsArray[i].enabled) {
                channelConfigs.push_back(channelConfigsArray[i]);
            }
        }
        
        sdWriter.setChannelNames(channelConfigs);
    }
    
    // Initialize the ADC
    baja::util::Debug::info(F("Initializing ADC..."));
    
    // Configure CS pin
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH);
    
    // Configure ADC settings
    baja::adc::ADCSettings adcSettings;
    adcSettings.deviceType = ID_AD7175_8;
    adcSettings.referenceSource = INTERNAL_REF;
    adcSettings.operatingMode = CONTINUOUS;
    adcSettings.readStatusWithData = true;
    adcSettings.odrSetting = SPS_2500;
    
    // Initialize ADC
    adcInitialized = adcHandler.begin(ADC_CS_PIN, SPI, adcSettings);
    
    if (!adcInitialized) {
        baja::util::Debug::error(F("ADC initialization failed!"));
        // Try a reset and reinitialize
        adcHandler.resetADC();
        delay(50);
        baja::util::Debug::info(F("Retrying ADC initialization..."));
        adcInitialized = adcHandler.begin(ADC_CS_PIN, SPI, adcSettings);
    }
    
    if (adcInitialized) {
        baja::util::Debug::info(F("ADC initialized successfully."));
        
        // Configure ADC channels
        baja::util::Debug::info(F("Configuring ADC channels..."));
        
        bool configSuccess = adcHandler.configureChannels(channelConfigsArray, baja::adc::ADC_CHANNEL_COUNT);
        
        if (!configSuccess) {
            baja::util::Debug::error(F("ADC channel configuration failed!"));
        } else {
            baja::util::Debug::info(F("ADC channels configured successfully."));
        }
    }
    
    // Start SD writer thread if SD card is initialized
    if (sdCardInitialized) {
        baja::util::Debug::info(F("Starting SD writer thread..."));
        
        sdWriterThreadId = threads.addThread(sdWriterThreadFunc, 0, baja::config::SD_WRITER_THREAD_STACK_SIZE);
        
        if (sdWriterThreadId < 0) {
            baja::util::Debug::error(F("Failed to create SD writer thread!"));
            sdWriterRunning = false;
        } else {
            baja::util::Debug::info(F("SD writer thread created with ID: ") + String(sdWriterThreadId));
        }
    } else {
        baja::util::Debug::warning(F("SD card not initialized, skipping SD writer thread"));
    }
    
    // Start ADC sampling if initialized
    if (adcInitialized) {
        baja::util::Debug::info(F("Starting ADC continuous sampling..."));
        if (!adcHandler.startSampling()) {
            baja::util::Debug::error(F("Failed to start ADC sampling!"));
            adcInitialized = false;
        } else {
            baja::util::Debug::info(F("ADC sampling started."));
            
            // Wait a bit and check for samples
            delay(100);
            adcHandler.pollForSample(10);
            
            baja::util::Debug::info(F("Sample count after startup: ") + String(adcHandler.getSampleCount()));
        }
    }

    // Network initialization for HTTP
    if (adcInitialized) {
        // Simple network initialization with static IP only
        baja::util::Debug::info("Initializing network with static IP...");

        // End any existing connections first to ensure clean state
        Ethernet.end();
        delay(100);

        // Initialize with static IP (no DHCP)
        IPAddress staticIP(192, 168, 20, 13);    // Use the IP that worked for you
        IPAddress gateway(192, 168, 20, 1);      // Your router's IP
        IPAddress subnet(255, 255, 255, 0);      // Standard subnet mask

        // Begin with static IP
        Ethernet.begin(staticIP, subnet, gateway);
        delay(100);

        // Log network status
        IPAddress ip = Ethernet.localIP();
        baja::util::Debug::info("Network initialized with IP: " + 
            String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]));

        // Initialize HTTP client with same server settings
        baja::util::Debug::info("Initializing HTTP client...");
        httpClient.begin("192.168.20.3", 9365, "/");

        // Start HTTP client thread
        baja::util::Debug::info("Starting HTTP client thread...");
        int httpClientThreadId = threads.addThread(httpClientThreadFunc, 0, 8192);
        if (httpClientThreadId < 0) {
            baja::util::Debug::error("Failed to create HTTP client thread!");
        } else {
            baja::util::Debug::info("HTTP client thread created with ID: " + String(httpClientThreadId));
        }
    }
    
    baja::util::Debug::info(F("Initialization complete. System running."));
}

void loop() {
    // Increment loop counter
    loopCount++;
    
    // Process ADC data using polling approach
    if (adcInitialized) {
        // Poll for new sample (non-blocking)
        if (adcHandler.pollForSample(0)) {
            samplesProcessedTotal++;
            
            // Print visual progress indicator
            if (samplesProcessedTotal % 10000 == 0) {
                Serial.print(".");
                
                if (samplesProcessedTotal % 500000 == 0) {
                    Serial.println();
                }
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
        if (sdWriterRunning && sdWriterThreadId > 0) {
            int threadState = threads.getState(sdWriterThreadId);
            
            // If thread has ended unexpectedly, restart it
            if (threadState != Threads::RUNNING) {
                baja::util::Debug::warning("SD Writer thread is not running! Attempting to restart...");
                
                sdWriterRunning = false;
                delay(100);
                
                // Create a new thread
                sdWriterThreadId = threads.addThread(sdWriterThreadFunc, 0, baja::config::SD_WRITER_THREAD_STACK_SIZE);
                
                if (sdWriterThreadId > 0) {
                    baja::util::Debug::info("SD Writer thread restarted with ID: " + String(sdWriterThreadId));
                    threads.setTimeSlice(sdWriterThreadId, baja::config::SD_WRITER_THREAD_PRIORITY);
                } else {
                    baja::util::Debug::error("Failed to restart SD Writer thread!");
                }
            }
        }
    }
    
    // Print status occasionally
    static uint32_t lastStatusTime = 0;
    
    // Print detailed status every 30 seconds
    if (currentTime - lastStatusTime > 30000) {
        lastStatusTime = currentTime;
        
        // Check for new samples
        uint32_t currentSampleCount = adcHandler.getSampleCount();
        uint32_t sampleDelta = currentSampleCount - lastSampleCount;
        baja::util::Debug::info("New samples in last 30 seconds: " + 
                           String(sampleDelta) + 
                           " (" + String(sampleDelta / 30.0f) + " samples/sec)");
        lastSampleCount = currentSampleCount;
        
        // Print buffer stats
        sampleBuffer.printStats();
        
        baja::util::Debug::info("\n========== SYSTEM STATUS ==========");
        baja::util::Debug::info("Uptime: " + String((currentTime - startTime) / 1000) + " seconds");
        baja::util::Debug::info("Loop count: " + String(loopCount));
        baja::util::Debug::info("Samples collected: " + String(adcHandler.getSampleCount()));
        baja::util::Debug::info("Active channel: " + String(adcHandler.getActiveChannel()));
        
        baja::util::Debug::info("Buffer usage: " + 
                         String(sampleBuffer.available()) + "/" + 
                         String(sampleBuffer.capacity()) + 
                         " (" + String((float)sampleBuffer.available() / sampleBuffer.capacity() * 100.0f) + "%)");
        
        if (sdCardInitialized) {
            baja::util::Debug::info("SD file: " + String(sdWriter.getCurrentFilename().c_str()));
            baja::util::Debug::info("SD bytes written: " + String(sdWriter.getBytesWritten()));
        }
        
        baja::util::Debug::info("===================================\n");
    }
    
    // Check if we have no samples after 10 seconds
    static bool noSamplesWarningPrinted = false;
    if (adcInitialized && currentTime - startTime > 10000 && adcHandler.getSampleCount() == 0 && !noSamplesWarningPrinted) {
        baja::util::Debug::warning("\n*** WARNING: No samples collected after 10 seconds! ***");
        baja::util::Debug::warning("Check ADC communication and waitForReady() operation");
        noSamplesWarningPrinted = true;
    }
    
    // Small yield for other tasks
    yield();
}