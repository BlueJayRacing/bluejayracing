#include <Arduino.h>
#include <TeensyThreads.h>
#include <SPI.h>
#include <TimeLib.h>

#include "ring_buffer.hpp"
#include "sample_data.hpp"
#include "adc_handler.hpp"
#include "sd_writer.hpp"
#include "mqtt_publisher.hpp"

// Pin definitions - UPDATED based on schematic
const uint8_t ADC_CS_PIN = 10;     // ADC chip select pin
const uint8_t ADC_DRDY_PIN = 24;   // ADC data ready pin (MISO2)
const uint8_t SD_CS_PIN = 254;     // SD card CS pin (for SPI fallback)

// Create global buffers in RAM2 to reduce RAM1 usage
DMAMEM baja::data::ChannelSample ringBufferStorage[baja::adc::RING_BUFFER_SIZE];
DMAMEM uint8_t sdWriterBuffer[baja::adc::SD_BLOCK_SIZE];
DMAMEM baja::data::ChannelSample sdSampleBuffer[baja::adc::SAMPLES_PER_SD_BLOCK];
DMAMEM uint8_t mqttMessageBuffer[baja::data::MQTT_OPTIMAL_MESSAGE_SIZE];
DMAMEM baja::adc::ChannelConfig channelConfigsArray[baja::adc::ADC_CHANNEL_COUNT];

// Create the global ring buffer (160KB) with external buffer
baja::buffer::RingBuffer<baja::data::ChannelSample, baja::adc::RING_BUFFER_SIZE> sampleBuffer(ringBufferStorage);

// Create the ADC handler
baja::adc::ADC7175Handler adcHandler(sampleBuffer);

// Create the SD writer
baja::storage::SDWriter sdWriter(sampleBuffer);

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

// Function to get time from Teensy RTC
time_t getTeensyTime() {
    return Teensy3Clock.get();
}

// Function to set up the correct time for the Teensy
void setupTime() {
    // Set a default time if no time is set (2024-01-01 00:00:00)
    if (year() < 2023) {
        setTime(0, 0, 0, 1, 1, 2024);
        Serial.println("Setting default time: 2024-01-01 00:00:00");
    }
    
    // Print current time
    char timeStr[32];
    sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
            year(), month(), day(), hour(), minute(), second());
    Serial.print("Current time: ");
    Serial.println(timeStr);
    
    // Sync with RTC
    setSyncProvider(getTeensyTime);
    
    // Check if RTC was set
    if (timeStatus() != timeSet) {
        Serial.println("RTC sync failed!");
    } else {
        Serial.println("RTC sync successful");
    }
}

// Function to check DRDY pin state
void checkDrdyPin() {
    Serial.print("DRDY pin state: ");
    Serial.println(digitalRead(ADC_DRDY_PIN) ? "HIGH" : "LOW");
}

// MQTT thread function
void mqttThreadFunc() {
    Serial.println("MQTT thread started");
    
    while (true) {
        // Check if MQTT is enabled
        if (mqttEnabled) {
            // Process MQTT messages
            size_t publishedCount = mqttPublisher.process();
            
            // Check if we lost connection
            if (!mqttPublisher.isConnected()) {
                Serial.println("MQTT connection lost, will attempt reconnect");
                mqttEnabled = false;
            }
        } else {
            // Check if it's time to attempt reconnection
            uint32_t now = millis();
            if (now - lastMqttReconnectAttempt > MQTT_RECONNECT_INTERVAL) {
                Serial.println("Attempting MQTT reconnection...");
                lastMqttReconnectAttempt = now;
                
                // Attempt to reconnect
                if (mqttPublisher.reconnect()) {
                    Serial.println("MQTT reconnected successfully");
                    mqttEnabled = true;
                } else {
                    Serial.println("MQTT reconnection failed, will try again later");
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
    Serial.println("SD Writer thread started");
    sdWriterRunning = true;
    
    // Create a new data file
    if (!sdWriter.createNewFile()) {
        Serial.println("SD Writer thread: Failed to create initial data file!");
    } else {
        Serial.print("SD Writer thread: Created initial data file: ");
        Serial.println(sdWriter.getCurrentFilename().c_str());
    }
    
    // Main SD writer loop
    uint32_t totalWritten = 0;
    while (sdWriterRunning) {
        // Process samples and write to SD card
        size_t samplesWritten = sdWriter.process();
        totalWritten += samplesWritten;
        
        // Print status occasionally
        static uint32_t lastStatusTime = 0;
        if (millis() - lastStatusTime > 5000) {
            Serial.print("SD Writer thread: Total samples written: ");
            Serial.print(totalWritten);
            Serial.print(", Current file: ");
            Serial.print(sdWriter.getCurrentFilename().c_str());
            Serial.print(", Bytes written: ");
            Serial.println(sdWriter.getBytesWritten());
            lastStatusTime = millis();
        }
        
        // Check if we need to rotate the file
        if (sdWriter.shouldRotateFile()) {
            Serial.println("SD Writer thread: Rotating file");
            sdWriter.closeFile();
            if (!sdWriter.createNewFile()) {
                Serial.println("SD Writer thread: Failed to create new file during rotation!");
            } else {
                Serial.print("SD Writer thread: Created new file: ");
                Serial.println(sdWriter.getCurrentFilename().c_str());
            }
        }
        
        // Small delay to prevent CPU hogging, but only if no samples were written
        if (samplesWritten == 0) {
            delay(1);
        }
        
        // Yield to other threads
        threads.yield();
    }
    
    // Close the file before exiting
    sdWriter.closeFile();
    Serial.println("SD Writer thread stopped");
}

// Initialize channel configurations
void initializeChannelConfigs() {
    Serial.println("Initializing channel configurations");
    
    // Set up 16 channels
    for (int i = 0; i < baja::adc::ADC_CHANNEL_COUNT; i++) {
        // Channel index
        channelConfigsArray[i].channelIndex = i;
        
        // Default to AINx for positive input and AINCOM for negative
        channelConfigsArray[i].analogInputs.ainp.pos_input = static_cast<ad717x_analog_input_t>(i);
        channelConfigsArray[i].analogInputs.ainp.neg_input = REF_M; // Use AIN1 as common negative
        
        // Default gain of 1
        channelConfigsArray[i].gain = 1.0;
        
        // Use setup 0 for all channels
        channelConfigsArray[i].setupIndex = 0;
        
        // Only enable channel 0 for initial testing
        channelConfigsArray[i].enabled = (i == 0);
        
        // Set channel name
        char name[16];
        snprintf(name, sizeof(name), "Channel_%d", i);
        channelConfigsArray[i].name = name;
        
        Serial.print("Configured channel ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(channelConfigsArray[i].name.c_str());
        Serial.print(" (");
        Serial.print(channelConfigsArray[i].enabled ? "enabled" : "disabled");
        Serial.println(")");
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
    
    Serial.println("Baja Data Acquisition System");
    Serial.println("Initializing...");
    
    // Set up the correct time first
    setupTime();
    
    // Initialize SPI early
    SPI.begin();
    
    // Initialize channel configurations
    initializeChannelConfigs();
    
    // Initialize the SD card
    Serial.println("Initializing SD card...");
    if (!(sdCardInitialized = sdWriter.begin())) {
        Serial.println("SD card initialization failed!");
    } else {
        Serial.println("SD card initialized.");
        
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
    Serial.println("Initializing ADC...");
    
    // Configure pins manually first
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH); // Deselect ADC
    pinMode(ADC_DRDY_PIN, INPUT_PULLUP);
    
    // Check DRDY pin state before reset
    checkDrdyPin();
    
    // Perform manual reset
    // adcHandler.resetADC();
    
    // Check DRDY pin state after reset
    // checkDrdyPin();
    
    // Configure ADC settings
    baja::adc::ADCSettings adcSettings;
    adcSettings.deviceType = ID_AD7175_8;
    adcSettings.referenceSource = INTERNAL_REF;
    adcSettings.operatingMode = CONTINUOUS;
    adcSettings.readStatusWithData = true;
    adcSettings.odrSetting = SPS_10; // Start with slower rate for testing
    
    // Initialize ADC
    adcInitialized = adcHandler.begin(ADC_CS_PIN, ADC_DRDY_PIN, SPI, adcSettings);
    
    if (!adcInitialized) {
        Serial.println("ADC initialization failed!");
        // Try a reset and init again
        adcHandler.resetADC();
        delay(50);
        Serial.println("Retrying ADC initialization...");
        adcInitialized = adcHandler.begin(ADC_CS_PIN, ADC_DRDY_PIN, SPI, adcSettings);
        
        if (!adcInitialized) {
            Serial.println("ADC initialization failed again!");
            // Continue anyway for debugging
        }
    }
    
    if (adcInitialized) {
        Serial.println("ADC initialized successfully.");
        
        // Configure ADC channels - only enable channel 0 for testing
        Serial.println("Configuring ADC channels...");
        
        // Create a smaller set of enabled channel configs for testing
        std::vector<baja::adc::ChannelConfig> enabledConfigs;
        for (int i = 0; i < baja::adc::ADC_CHANNEL_COUNT; i++) {
            if (channelConfigsArray[i].enabled) {
                enabledConfigs.push_back(channelConfigsArray[i]);
            }
        }
        
        bool configSuccess = false;
        if (!enabledConfigs.empty()) {
            // Configure only enabled channels
            configSuccess = adcHandler.configureChannels(&enabledConfigs[0], enabledConfigs.size());
        } else {
            Serial.println("No enabled channels found!");
        }
        
        if (!configSuccess) {
            Serial.println("ADC channel configuration failed!");
        } else {
            Serial.println("ADC channels configured successfully.");
        }
        
        // Set interrupt priority (just below maximum)
        adcHandler.setInterruptPriority(64); // Lower number = higher priority
    }
    
    // // Start SD writer thread if SD card is initialized
    if (sdCardInitialized) {
        Serial.println("Starting SD writer thread...");
        threads.addThread(sdWriterThreadFunc);
    } else {
        Serial.println("SD card not initialized, skipping SD writer thread");
    }
    
    // Start ADC sampling if initialized
    if (adcInitialized) {
        Serial.println("Starting ADC continuous sampling...");
        if (!adcHandler.startSampling()) {
            Serial.println("Failed to start ADC sampling!");
            adcInitialized = false;
        } else {
            Serial.println("ADC sampling started.");
            
            // Check DRDY pin state after starting sampling
            checkDrdyPin();
            
            // Wait a bit and print sample count to confirm it's working
            delay(100);
            Serial.print("Sample count after startup: ");
            Serial.println(adcHandler.getSampleCount());
        }
    }
    
    Serial.println("Initialization complete. System running.");
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
            
            // Print a dot every 1000 samples for visual feedback
            if (samplesProcessedTotal % 1000 == 0) {
                Serial.print(".");
                
                if (samplesProcessedTotal % 50000 == 0) {
                    Serial.println();
                }
            }
        }
    }
    
    // Print status occasionally
    static uint32_t lastStatusTime = 0;
    static uint32_t lastDrdyCheckTime = 0;
    static uint32_t lastBufferPrintTime = 0;
    uint32_t currentTime = millis();
    
    // Check DRDY pin state every 10 seconds
    if (currentTime - lastDrdyCheckTime > 10000) {
        checkDrdyPin();
        lastDrdyCheckTime = currentTime;
        
        // Also check for new samples
        uint32_t currentSampleCount = adcHandler.getSampleCount();
        uint32_t sampleDelta = currentSampleCount - lastSampleCount;
        Serial.print("New samples in last 10 seconds: ");
        Serial.print(sampleDelta);
        Serial.print(" (");
        Serial.print(sampleDelta / 10.0f);
        Serial.println(" Hz)");
        lastSampleCount = currentSampleCount;
    }
    
    // Print buffer stats every 5 seconds
    if (currentTime - lastBufferPrintTime > 5000) {
        sampleBuffer.printStats();
        lastBufferPrintTime = currentTime;
    }
    
    // Print detailed status every 10 seconds
    if (currentTime - lastStatusTime > 10000) {
        Serial.println("\n========== SYSTEM STATUS ==========");
        Serial.print("Uptime: ");
        Serial.print((currentTime - startTime) / 1000);
        Serial.println(" seconds");
        Serial.print("Loop count: ");
        Serial.println(loopCount);
        
        // Print current time
        char timeStr[32];
        sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
                year(), month(), day(), hour(), minute(), second());
        Serial.print("Current time: ");
        Serial.println(timeStr);
        
        Serial.println("----- ADC Status -----");
        Serial.print("  ADC initialized: ");
        Serial.println(adcInitialized ? "Yes" : "No");
        Serial.print("  Samples collected: ");
        Serial.println(adcHandler.getSampleCount());
        Serial.print("  Sampling rate: ");
        float samplingRate = 0;
        if (currentTime > startTime) {
            samplingRate = (float)adcHandler.getSampleCount() * 1000 / (currentTime - startTime);
        }
        Serial.print(samplingRate);
        Serial.println(" Hz");
        Serial.print("  Active channel: ");
        Serial.println(adcHandler.getActiveChannel());
        
        Serial.println("----- Buffer Status -----");
        Serial.print("  Buffer usage: ");
        Serial.print(sampleBuffer.available());
        Serial.print("/");
        Serial.print(sampleBuffer.capacity());
        Serial.print(" (");
        Serial.print((float)sampleBuffer.available() / sampleBuffer.capacity() * 100.0f);
        Serial.println("%)");
        
        Serial.println("----- SD Card Status -----");
        Serial.print("  SD card initialized: ");
        Serial.println(sdCardInitialized ? "Yes" : "No");
        Serial.print("  SD writer thread running: ");
        Serial.println(sdWriterRunning ? "Yes" : "No");
        if (sdCardInitialized) {
            Serial.print("  Current file: ");
            Serial.println(sdWriter.getCurrentFilename().c_str());
            Serial.print("  Bytes written: ");
            Serial.println(sdWriter.getBytesWritten());
        }
        
        Serial.println("----- MQTT Status -----");
        Serial.print("  MQTT connected: ");
        Serial.println(mqttEnabled ? "Yes" : "No");
        if (!mqttEnabled) {
            Serial.print("  Next reconnect attempt: ");
            int32_t timeToReconnect = MQTT_RECONNECT_INTERVAL - (currentTime - lastMqttReconnectAttempt);
            Serial.print(timeToReconnect > 0 ? timeToReconnect / 1000 : 0);
            Serial.println(" seconds");
        }
        
        Serial.println("===================================\n");
        lastStatusTime = currentTime;
    }
    
    // Check if we have no samples after a certain time
    static bool noSamplesWarningPrinted = false;
    if (adcInitialized && currentTime - startTime > 10000 && adcHandler.getSampleCount() == 0 && !noSamplesWarningPrinted) {
        Serial.println("\n*** WARNING: No samples collected after 10 seconds! ***");
        Serial.println("Possible issues:");
        Serial.println("1. DRDY pin not connected correctly");
        Serial.println("2. ADC not in continuous conversion mode");
        Serial.println("3. Interrupt not being triggered");
        noSamplesWarningPrinted = true;
    }
    
    // Yield a small amount of time to allow other tasks to run if needed
    yield();
}