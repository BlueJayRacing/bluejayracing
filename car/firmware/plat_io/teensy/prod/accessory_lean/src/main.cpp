#include <Arduino.h>
#include <TeensyThreads.h>
#include <SPI.h>
#include <TimeLib.h>

#include "ring_buffer.hpp"
#include "sample_data.hpp"
#include "adc_handler.hpp"
#include "sd_writer.hpp"
#include "mqtt_publisher.hpp"

// Pin definitions
const uint8_t ADC_CS_PIN = 10;       // ADC chip select pin
const uint8_t ADC_DRDY_PIN = 9;      // ADC data ready pin
const uint8_t SD_CS_PIN = 254; // SD card CS pin (for SPI fallback)

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

// Initialize channel configurations
void initializeChannelConfigs() {
    Serial.println("Initializing channel configurations");
    
    // Set up 16 channels
    for (int i = 0; i < baja::adc::ADC_CHANNEL_COUNT; i++) {
        // Channel index
        channelConfigsArray[i].channelIndex = i;
        
        // Default to AINx for positive input and AINCOM for negative
        channelConfigsArray[i].analogInputs.ainp.pos_input = static_cast<ad717x_analog_input_t>(i);
        channelConfigsArray[i].analogInputs.ainp.neg_input = AIN1; // Use AIN1 as common negative
        
        // Default gain of 1
        channelConfigsArray[i].gain = 1.0;
        
        // Use setup 0 for all channels
        channelConfigsArray[i].setupIndex = 0;
        
        // Enable only first 4 channels for now (for debugging)
        channelConfigsArray[i].enabled = (i < 4);
        
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

// Function to get time from Teensy RTC
time_t getTeensyTime() {
  return (time_t)Teensy3Clock.get();
}

FLASHMEM void setup() {
    // Initialize serial
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for serial monitor to connect
    }
    
    Serial.println("Baja Data Acquisition System");
    Serial.println("Initializing...");
    
    // Set up the RTC (for file timestamps)
    setSyncProvider(getTeensyTime);
    
    // Initialize channel configurations
    initializeChannelConfigs();
    
    // Initialize the SD card
    Serial.println("Initializing SD card...");
    if (!sdWriter.begin()) {
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
    baja::adc::ADCSettings adcSettings;
    adcSettings.deviceType = ID_AD7175_8;
    adcSettings.referenceSource = INTERNAL_REF;
    adcSettings.operatingMode = CONTINUOUS;
    adcSettings.readStatusWithData = true;
    adcSettings.odrSetting = SPS_50000; // 50kHz total sampling rate
    
    // Make sure SPI is initialized before configuring the ADC
    SPI.begin();
    
    // Initialize ADC without try/catch
    bool adcInitialized = adcHandler.begin(ADC_CS_PIN, ADC_DRDY_PIN, SPI, adcSettings);
    
    if (!adcInitialized) {
        Serial.println("ADC initialization failed!");
        // Continue anyway for debugging
    } else {
        Serial.println("ADC initialized.");
        
        // Configure ADC channels - only enable the first 4 for testing
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
            Serial.println("ADC channels configured.");
        }
        
        // Set interrupt priority (just below maximum)
        adcHandler.setInterruptPriority(16); // Lower number = higher priority
    }
    
    // Initialize MQTT publisher with timeout
    Serial.println("Initializing MQTT publisher...");
    
    // Set a timeout for MQTT initialization
    uint32_t mqttStartTime = millis();
    uint32_t mqttTimeout = 2000; // 2 second timeout
    
    // Use a non-blocking approach to MQTT initialization
    mqttPublisher.setChannelConfigs(std::vector<baja::adc::ChannelConfig>(channelConfigsArray, 
                                                                         channelConfigsArray + baja::adc::ADC_CHANNEL_COUNT));
    mqttPublisher.setDownsampleRatio(10); // Send 1/10th of the samples over MQTT
    
    // Start MQTT thread regardless of connection status
    Serial.println("Starting MQTT thread...");
    std::thread mqttThread(mqttThreadFunc);
    mqttThread.detach();
    
    // Initialize MQTT with short timeout
    Serial.println("Attempting MQTT connection (non-blocking)...");
    lastMqttReconnectAttempt = millis();
    mqttEnabled = mqttPublisher.begin(nullptr, "TeensyDAQ", "192.168.1.100", 1883);
    
    if (!mqttEnabled) {
        Serial.println("MQTT initialization deferred - will retry in background");
    } else {
        Serial.println("MQTT initialized and connected successfully");
    }
    
    // Create new SD card file
    if (!sdWriter.createNewFile()) {
        Serial.println("Failed to create data file!");
    } else {
        Serial.print("Created data file: ");
        Serial.println(sdWriter.getCurrentFilename().c_str());
    }
    
    // Start ADC sampling (if ADC initialized)
    if (adcInitialized) {
        Serial.println("Starting ADC sampling...");
        if (!adcHandler.startSampling()) {
            Serial.println("Failed to start ADC sampling!");
        } else {
            Serial.println("ADC sampling started.");
        }
    }
    
    Serial.println("Initialization complete.");
}

void loop() {
    // Process samples and write to SD card
    size_t samplesWritten = sdWriter.process();
    
    // Print status occasionally
    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime > 5000) {
        Serial.println("Status:");
        Serial.print("  Samples collected: ");
        Serial.println(adcHandler.getSampleCount());
        Serial.print("  Buffer usage: ");
        Serial.print(sampleBuffer.available());
        Serial.print("/");
        Serial.print(sampleBuffer.capacity());
        Serial.print(" (");
        Serial.print((float)sampleBuffer.available() / sampleBuffer.capacity() * 100.0f);
        Serial.println("%)");
        Serial.print("  Current file: ");
        Serial.println(sdWriter.getCurrentFilename().c_str());
        Serial.print("  Bytes written: ");
        Serial.println(sdWriter.getBytesWritten());
        Serial.print("  MQTT connected: ");
        Serial.println(mqttEnabled ? "Yes" : "No");
        
        lastStatusTime = millis();
    }
    
    // Small delay to allow other tasks to run
    if (samplesWritten == 0) {
        delay(1);
    }
}