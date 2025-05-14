#include <Arduino.h>
#include <TeensyThreads.h>
#include <SPI.h>
#include <TimeLib.h>

#include "ring_buffer.hpp"
#include "sample_data.hpp"
#include "adc_handler.hpp"
#include "sd_writer.hpp"
#include "mqtt_publisher.hpp"
#include "config.hpp"
#include "debug_util.hpp"

// Pin definitions - UPDATED based on schematic
const uint8_t ADC_CS_PIN = 10;     // ADC chip select pin
const uint8_t ADC_DRDY_PIN = 24;   // ADC data ready pin (MISO2)
const uint8_t SD_CS_PIN = 254;     // SD card CS pin (for SPI fallback)

// Create global buffers in RAM2 to reduce RAM1 usage
EXTMEM baja::data::ChannelSample ringBufferStorage[baja::config::SAMPLE_RING_BUFFER_SIZE];
EXTMEM baja::adc::ChannelConfig channelConfigsArray[baja::adc::ADC_CHANNEL_COUNT];
EXTMEM uint8_t mqttMessageBuffer[baja::data::MQTT_OPTIMAL_MESSAGE_SIZE];

// Define the SdFat RingBuf in EXTMEM
// Note: RingBuf allocates its own internal buffer, so we need to define the entire object in EXTMEM
DMAMEM RingBuf<FsFile, baja::config::SD_RING_BUF_CAPACITY> sdRingBuf;

// Create the global ring buffer with external buffer
baja::buffer::RingBuffer<baja::data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE> sampleBuffer(ringBufferStorage);

// Create the ADC handler
baja::adc::ADC7175Handler adcHandler(sampleBuffer);

// Create the SD writer with external RingBuf
baja::storage::SDWriter sdWriter(sampleBuffer, &sdRingBuf);

// Create the MQTT publisher
baja::network::MQTTPublisher mqttPublisher(sampleBuffer);

// Global flag for MQTT state
volatile bool mqttEnabled = false;
volatile uint32_t lastMqttReconnectAttempt = 0;
const uint32_t MQTT_RECONNECT_INTERVAL = 30000; // 30 seconds between reconnection attempts

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
    // Sync with RTC
    setSyncProvider(getTeensyTime);  
    SdFile::dateTimeCallback(dateTime);

    // Set a default time if no time is set (2024-01-01 00:00:00)
    if (year() < 2023) {
        setTime(0, 0, 0, 1, 1, 2024);
        baja::util::Debug::info("Setting default time: 2024-01-01 00:00:00");
    }
    
    // Print current time
    char timeStr[32];
    sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
            year(), month(), day(), hour(), minute(), second());
    String currentTimeStr = "Current time: " + String(timeStr);
    baja::util::Debug::info(currentTimeStr.c_str());
    
    // Check if RTC was set
    if (timeStatus() != timeSet) {
        baja::util::Debug::warning("RTC sync failed!");
    } else {
        baja::util::Debug::info("RTC sync successful");
    }
}

// Function to check DRDY pin state
void checkDrdyPin() {
    String pinState = "DRDY pin state: " + String(digitalRead(ADC_DRDY_PIN) ? "HIGH" : "LOW");
    baja::util::Debug::info(pinState.c_str());
}

// MQTT thread function
void mqttThreadFunc() {
    baja::util::Debug::info("MQTT thread started");
    
    while (true) {
        // Check if MQTT is enabled
        if (mqttEnabled) {
            // Process MQTT messages
            mqttPublisher.process();
            
            // Check if we lost connection
            if (!mqttPublisher.isConnected()) {
                baja::util::Debug::warning("MQTT connection lost, will attempt reconnect");
                mqttEnabled = false;
            }
        } else {
            // Check if it's time to attempt reconnection
            uint32_t now = millis();
            if (now - lastMqttReconnectAttempt > MQTT_RECONNECT_INTERVAL) {
                baja::util::Debug::info("Attempting MQTT reconnection...");
                lastMqttReconnectAttempt = now;
                
                // Attempt to reconnect
                if (mqttPublisher.reconnect()) {
                    baja::util::Debug::info("MQTT reconnected successfully");
                    mqttEnabled = true;
                } else {
                    baja::util::Debug::warning("MQTT reconnection failed, will try again later");
                }
            }
        }
        
        // Yield to other threads
        threads.yield();
        
        // Small delay to prevent CPU hogging
        delay(10);
    }
}

// SD writer thread function
void sdWriterThreadFunc() {
    baja::util::Debug::info("SD Writer thread started");
    sdWriterRunning = true;
    
    // Set a longer time slice for this thread
    threads.setTimeSlice(threads.id(), baja::config::SD_WRITER_THREAD_PRIORITY);
    
    // Create a new data file
    if (!sdWriter.createNewFile()) {
        baja::util::Debug::error("SD Writer thread: Failed to create initial data file!");
    } else {
        baja::util::Debug::info("SD Writer thread: Created initial data file: " + String(sdWriter.getCurrentFilename().c_str()));
    }
    
    // Main SD writer loop
    uint32_t totalWritten = 0;
    uint32_t lastStatusTime = 0;
    uint32_t lastAliveMessage = 0;
    
    while (sdWriterRunning) {
        // Process samples and write to SD card
        size_t samplesWritten = sdWriter.process();
        totalWritten += samplesWritten;
        
        // Print status occasionally (every 30 seconds)
        uint32_t now = millis();
        if (now - lastStatusTime > 30000) {
            String statusStr = "SD Writer thread: Total samples written: " + 
                               String(totalWritten) + 
                               ", Current file: " + 
                               String(sdWriter.getCurrentFilename().c_str()) + 
                               ", Bytes written: " + 
                               String(sdWriter.getBytesWritten());
            baja::util::Debug::info(statusStr.c_str());
            lastStatusTime = now;
        }
        
        // Always print an alive message every minute
        if (now - lastAliveMessage > 60000) {
            baja::util::Debug::info("SD Writer thread: Still alive and running");
            lastAliveMessage = now;
        }
        
        // Check if writer is still healthy
        if (!sdWriter.isHealthy()) {
            baja::util::Debug::warning("SD Writer thread: Detected unhealthy state, attempting recovery");
            
            // Try to close and recreate the file
            sdWriter.closeFile();
            delay(100); // Brief delay for recovery
            
            if (sdWriter.createNewFile()) {
                baja::util::Debug::info("SD Writer thread: Recovery successful");
                sdWriter.resetHealth();
            } else {
                baja::util::Debug::warning("SD Writer thread: Recovery failed, will retry later");
                delay(5000); // Longer delay before next retry
            }
        }
        
        // Small yield for other threads
        threads.yield();
    }
    
    // Close the file before exiting
    sdWriter.closeFile();
    baja::util::Debug::info("SD Writer thread stopped");
}

// Initialize channel configurations
void initializeChannelConfigs() {
    baja::util::Debug::info("Initializing channel configurations");
    
    // Set up all 16 channels
    for (int i = 0; i < baja::adc::ADC_CHANNEL_COUNT; i++) {
        // Channel index
        channelConfigsArray[i].channelIndex = i;
        
        // Default to AINx for positive input and AINCOM for negative
        channelConfigsArray[i].analogInputs.ainp.pos_input = static_cast<ad717x_analog_input_t>(i);
        channelConfigsArray[i].analogInputs.ainp.neg_input = REF_M; // Use reference as common negative
        
        // Default gain of 1
        channelConfigsArray[i].gain = baja::config::ADC_DEFAULT_GAIN;
        
        // Use setup 0 for all channels
        channelConfigsArray[i].setupIndex = 0;
        
        // Enable all channels if configured, otherwise just enable 0 and 1 for testing
        channelConfigsArray[i].enabled = baja::config::ADC_ENABLE_ALL_CHANNELS ? true : (i < 2);
        
        // Set channel name
        char name[16];
        snprintf(name, sizeof(name), "Channel_%d", i);
        channelConfigsArray[i].name = name;
        
        baja::util::Debug::detail("Configured channel " + String(i) + 
                               ": " + String(channelConfigsArray[i].name.c_str()) + 
                               " (" + String(channelConfigsArray[i].enabled ? "enabled" : "disabled") + ")");
    }
}

void setup() {
    // Initialize serial
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for serial monitor to connect
    }
    
    // Record start time
    startTime = millis();
    
    // Initialize debug utility
    baja::util::Debug::init(baja::util::Debug::INFO);
    
    baja::util::Debug::info("Baja Data Acquisition System");
    baja::util::Debug::info("Initializing...");
    
    // Set up the correct time first
    setupTime();
    
    // Initialize SPI early
    SPI.begin();
    
    // Set up microslices for faster thread switching
    threads.setSliceMicros(baja::config::THREAD_SLICE_MICROS);
    
    // Initialize channel configurations
    initializeChannelConfigs();

    // Initialize the SD card
    baja::util::Debug::info("Initializing SD card...");
    if (!(sdCardInitialized = sdWriter.begin())) {
        baja::util::Debug::error("SD card initialization failed!");
    } else {
        baja::util::Debug::info("SD card initialized.");
        
        // Create vector of all channel configs for the SD writer
        std::vector<baja::adc::ChannelConfig> channelConfigs;
        for (int i = 0; i < baja::adc::ADC_CHANNEL_COUNT; i++) {
            if (channelConfigsArray[i].enabled) {
                channelConfigs.push_back(channelConfigsArray[i]);
            }
        }
        
        sdWriter.setChannelNames(channelConfigs);
    }
    
    // Initialize the ADC
    baja::util::Debug::info("Initializing ADC...");
    
    // Configure pins manually first
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH); // Deselect ADC
    pinMode(ADC_DRDY_PIN, INPUT_PULLUP);
    
    // Check DRDY pin state before reset
    checkDrdyPin();
    
    // Configure ADC settings
    baja::adc::ADCSettings adcSettings;
    adcSettings.deviceType = ID_AD7175_8;
    adcSettings.referenceSource = INTERNAL_REF;
    adcSettings.operatingMode = CONTINUOUS;
    adcSettings.readStatusWithData = true;
    adcSettings.odrSetting = SPS_50000; // Try a lower rate for testing
    
    // Initialize ADC
    adcInitialized = adcHandler.begin(ADC_CS_PIN, ADC_DRDY_PIN, SPI, adcSettings);
    
    if (!adcInitialized) {
        baja::util::Debug::error("ADC initialization failed!");
        // Try a reset and init again
        adcHandler.resetADC();
        delay(50);
        baja::util::Debug::info("Retrying ADC initialization...");
        adcInitialized = adcHandler.begin(ADC_CS_PIN, ADC_DRDY_PIN, SPI, adcSettings);
        
        if (!adcInitialized) {
            baja::util::Debug::error("ADC initialization failed again!");
        }
    }
    
    if (adcInitialized) {
        baja::util::Debug::info("ADC initialized successfully.");
        
        // Configure ADC channels
        baja::util::Debug::info("Configuring ADC channels...");
        
        // Configure channels
        bool configSuccess = adcHandler.configureChannels(channelConfigsArray, baja::adc::ADC_CHANNEL_COUNT);
        
        if (!configSuccess) {
            baja::util::Debug::error("ADC channel configuration failed!");
        } else {
            baja::util::Debug::info("ADC channels configured successfully.");
        }
        
        // Set interrupt priority (just below maximum)
        adcHandler.setInterruptPriority(64); // Lower number = higher priority
    }
    
    // Start SD writer thread if SD card is initialized
    if (sdCardInitialized) {
        baja::util::Debug::info("Starting SD writer thread...");
        
        // Use a larger stack and longer time slice for SD writer thread
        sdWriterThreadId = threads.addThread(sdWriterThreadFunc, 0, baja::config::SD_WRITER_THREAD_STACK_SIZE);
        
        if (sdWriterThreadId < 0) {
            baja::util::Debug::error("Failed to create SD writer thread!");
            sdWriterRunning = false;
        } else {
            String idStr = "SD writer thread created with ID: " + String(sdWriterThreadId);
            baja::util::Debug::info(idStr.c_str());
        }
    } else {
        baja::util::Debug::warning("SD card not initialized, skipping SD writer thread");
    }
    
    // Start ADC sampling if initialized
    if (adcInitialized) {
        baja::util::Debug::info("Starting ADC continuous sampling...");
        if (!adcHandler.startSampling()) {
            baja::util::Debug::error("Failed to start ADC sampling!");
            adcInitialized = false;
        } else {
            baja::util::Debug::info("ADC sampling started.");
            
            // Check DRDY pin state after starting sampling
            checkDrdyPin();
            
            // Wait a bit and print sample count to confirm it's working
            delay(100);
            String countStr = "Sample count after startup: " + String(adcHandler.getSampleCount());
            baja::util::Debug::info(countStr.c_str());
        }
    }
    
    baja::util::Debug::info("Initialization complete. System running.");
    adcHandler.signalDataReady();
}

void loop() {
    // Increment loop counter
    loopCount++;
    
    // Process ADC data (using flag approach)
    if (adcInitialized) {
        // Process ADC data when available
        if (adcHandler.processData()) {
            samplesProcessedTotal++;
            
            // Print a dot every 10000 samples for visual feedback
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
        
        // Check SD writer thread health if it should be running
        if (sdWriterRunning && sdWriterThreadId > 0) {
            int threadState = threads.getState(sdWriterThreadId);
            
            // Print thread state for debugging
            String stateStr = "SD Writer thread state: " + String(threadState);
            baja::util::Debug::detail(stateStr.c_str());
            
            // If thread has ended unexpectedly, restart it
            if (threadState != Threads::RUNNING) {
                baja::util::Debug::warning("SD Writer thread is not running! Attempting to restart...");
                
                // Reset the flag first
                sdWriterRunning = false;
                delay(100);
                
                // Create a new thread
                sdWriterThreadId = threads.addThread(sdWriterThreadFunc, 0, baja::config::SD_WRITER_THREAD_STACK_SIZE);
                
                if (sdWriterThreadId > 0) {
                    String restartStr = "SD Writer thread restarted with ID: " + String(sdWriterThreadId);
                    baja::util::Debug::info(restartStr.c_str());
                    threads.setTimeSlice(sdWriterThreadId, baja::config::SD_WRITER_THREAD_PRIORITY);
                } else {
                    baja::util::Debug::error("Failed to restart SD Writer thread!");
                }
            }
        }
    }
    
    // Print status occasionally
    static uint32_t lastStatusTime = 0;
    static uint32_t lastDrdyCheckTime = 0;
    static uint32_t lastBufferPrintTime = 0;
    
    // Check DRDY pin state every 30 seconds
    if (currentTime - lastDrdyCheckTime > 30000) {
        checkDrdyPin();
        lastDrdyCheckTime = currentTime;
        
        // Also check for new samples
        uint32_t currentSampleCount = adcHandler.getSampleCount();
        uint32_t sampleDelta = currentSampleCount - lastSampleCount;
        String sampleStr = "New samples in last 30 seconds: " + 
                          String(sampleDelta) + 
                          " (" + String(sampleDelta / 30.0f) + " Hz)";
        baja::util::Debug::info(sampleStr.c_str());
        lastSampleCount = currentSampleCount;
    }
    
    // Print buffer stats every 15 seconds
    if (currentTime - lastBufferPrintTime > 15000) {
        sampleBuffer.printStats();
        lastBufferPrintTime = currentTime;
    }
    
    // Print detailed status every 30 seconds
    if (currentTime - lastStatusTime > 30000) {
        baja::util::Debug::info("\n========== SYSTEM STATUS ==========");
        
        String uptimeStr = "Uptime: " + String((currentTime - startTime) / 1000) + " seconds";
        baja::util::Debug::info(uptimeStr.c_str());
        
        String loopStr = "Loop count: " + String(loopCount);
        baja::util::Debug::info(loopStr.c_str());
        
        // Print current time
        char timeStr[32];
        sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
                year(), month(), day(), hour(), minute(), second());
        String currentTimeStr = "Current time: " + String(timeStr);
        baja::util::Debug::info(currentTimeStr.c_str());
        
        baja::util::Debug::info("----- ADC Status -----");
        
        String adcInitStr = "  ADC initialized: " + String(adcInitialized ? "Yes" : "No");
        baja::util::Debug::info(adcInitStr.c_str());
        
        String samplesStr = "  Samples collected: " + String(adcHandler.getSampleCount());
        baja::util::Debug::info(samplesStr.c_str());
        
        String channelStr = "  Active channel: " + String(adcHandler.getActiveChannel());
        baja::util::Debug::info(channelStr.c_str());
        
        baja::util::Debug::info("----- Buffer Status -----");
        
        String bufferStr = "  Buffer usage: " + 
                          String(sampleBuffer.available()) + "/" + 
                          String(sampleBuffer.capacity()) + 
                          " (" + String((float)sampleBuffer.available() / sampleBuffer.capacity() * 100.0f) + "%)";
        baja::util::Debug::info(bufferStr.c_str());
        
        baja::util::Debug::info("----- SD Card Status -----");
        
        String sdInitStr = "  SD card initialized: " + String(sdCardInitialized ? "Yes" : "No");
        baja::util::Debug::info(sdInitStr.c_str());
        
        String threadStr = "  SD writer thread running: " + String(sdWriterRunning ? "Yes" : "No");
        baja::util::Debug::info(threadStr.c_str());
        
        if (sdCardInitialized) {
            String fileStr = "  Current file: " + String(sdWriter.getCurrentFilename().c_str());
            baja::util::Debug::info(fileStr.c_str());
            
            String bytesStr = "  Bytes written: " + String(sdWriter.getBytesWritten());
            baja::util::Debug::info(bytesStr.c_str());
            
            String healthStr = "  SD writer healthy: " + String(sdWriter.isHealthy() ? "Yes" : "No");
            baja::util::Debug::info(healthStr.c_str());
        }
        
        baja::util::Debug::info("----- Thread Status -----");
        baja::util::Debug::info(String(threads.threadsInfo()));
        
        baja::util::Debug::info("===================================\n");
        lastStatusTime = currentTime;
    }
    
    // Check if we have no samples after a certain time
    static bool noSamplesWarningPrinted = false;
    if (adcInitialized && currentTime - startTime > 10000 && adcHandler.getSampleCount() == 0 && !noSamplesWarningPrinted) {
        baja::util::Debug::warning("\n*** WARNING: No samples collected after 10 seconds! ***");
        baja::util::Debug::warning("Possible issues:");
        baja::util::Debug::warning("1. DRDY pin not connected correctly");
        baja::util::Debug::warning("2. ADC not in continuous conversion mode");
        baja::util::Debug::warning("3. Interrupt not being triggered");
        noSamplesWarningPrinted = true;
    }
    
    // Yield a small amount of time to allow other tasks to run if needed
    yield();
}