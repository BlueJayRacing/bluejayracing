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
DMAMEM uint8_t ringBufferStorage[baja::adc::RING_BUFFER_SIZE * sizeof(baja::data::ChannelSample)];
DMAMEM uint8_t sdWriterBuffer[baja::adc::SD_BLOCK_SIZE];
DMAMEM baja::data::ChannelSample sdSampleBuffer[baja::adc::SAMPLES_PER_SD_BLOCK];
DMAMEM uint8_t mqttMessageBuffer[baja::data::MQTT_OPTIMAL_MESSAGE_SIZE];
DMAMEM baja::adc::ChannelConfig channelConfigsArray[baja::adc::ADC_CHANNEL_COUNT];

// Create the global ring buffer (160KB)
baja::buffer::RingBuffer<baja::data::ChannelSample, baja::adc::RING_BUFFER_SIZE> sampleBuffer;

// Create the ADC handler
baja::adc::ADC7175Handler adcHandler(sampleBuffer);

// Create the SD writer
baja::storage::SDWriter sdWriter(sampleBuffer);

// Create the MQTT publisher
baja::network::MQTTPublisher mqttPublisher(sampleBuffer);

// Channel configuration
baja::adc::ChannelConfig* channelConfigs = channelConfigsArray;

// MQTT thread function
void mqttThreadFunc() {
    Serial.println("MQTT thread started");
    
    while (true) {
        // Process MQTT messages
        size_t publishedCount = mqttPublisher.process();
        
        // Yield to other threads
        threads.yield();
        
        // Small delay to prevent CPU hogging
        if (publishedCount == 0) {
            delay(10);
        }
    }
}

// Initialize channel configurations
void initializeChannelConfigs() {
    // Set up 16 channels
    for (int i = 0; i < baja::adc::ADC_CHANNEL_COUNT; i++) {
        // Channel index
        channelConfigs[i].channelIndex = i;
        
        // Default to AINx for positive input and AINCOM for negative
        channelConfigs[i].analogInputs.ainp.pos_input = static_cast<ad717x_analog_input_t>(i);
        channelConfigs[i].analogInputs.ainp.neg_input = AIN1; // Use AIN1 as common negative
        
        // Default gain of 1
        channelConfigs[i].gain = 1.0;
        
        // Use setup 0 for all channels
        channelConfigs[i].setupIndex = 0;
        
        // Enable all channels
        channelConfigs[i].enabled = true;
        
        // Set channel name
        char name[16];
        sprintf(name, "Channel %d", i);
        channelConfigs[i].name = name;
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
        sdWriter.setChannelNames(std::vector<baja::adc::ChannelConfig>(channelConfigs, 
                                                                    channelConfigs + baja::adc::ADC_CHANNEL_COUNT));
    }
    
    // Initialize the ADC
    Serial.println("Initializing ADC...");
    baja::adc::ADCSettings adcSettings;
    adcSettings.deviceType = ID_AD7175_8;
    adcSettings.referenceSource = INTERNAL_REF;
    adcSettings.operatingMode = CONTINUOUS;
    adcSettings.readStatusWithData = true;
    adcSettings.odrSetting = SPS_50000; // 50kHz total sampling rate
    
    if (!adcHandler.begin(ADC_CS_PIN, ADC_DRDY_PIN, SPI, adcSettings)) {
        Serial.println("ADC initialization failed!");
    } else {
        Serial.println("ADC initialized.");
        
        // Configure ADC channels
        if (!adcHandler.configureChannels(channelConfigs, baja::adc::ADC_CHANNEL_COUNT)) {
            Serial.println("ADC channel configuration failed!");
        } else {
            Serial.println("ADC channels configured.");
        }
        
        // Set interrupt priority (just below maximum)
        adcHandler.setInterruptPriority(16); // Lower number = higher priority
    }
    
    // Initialize MQTT publisher
    Serial.println("Initializing MQTT publisher...");
    if (!mqttPublisher.begin(nullptr, "TeensyDAQ", "192.168.1.100")) {
        Serial.println("MQTT initialization failed!");
    } else {
        Serial.println("MQTT initialized.");
        mqttPublisher.setChannelConfigs(std::vector<baja::adc::ChannelConfig>(channelConfigs, 
                                                                           channelConfigs + baja::adc::ADC_CHANNEL_COUNT));
        mqttPublisher.setDownsampleRatio(10); // Send 1/10th of the samples over MQTT
    }
    
    // Create new SD card file
    if (!sdWriter.createNewFile()) {
        Serial.println("Failed to create data file!");
    } else {
        Serial.print("Created data file: ");
        Serial.println(sdWriter.getCurrentFilename().c_str());
    }
    
    // Start the MQTT thread
    std::thread mqttThread(mqttThreadFunc);
    mqttThread.detach();
    
    // Start ADC sampling
    Serial.println("Starting ADC sampling...");
    if (!adcHandler.startSampling()) {
        Serial.println("Failed to start ADC sampling!");
    } else {
        Serial.println("ADC sampling started.");
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
        
        lastStatusTime = millis();
    }
    
    // Small delay to allow other tasks to run
    if (samplesWritten == 0) {
        delay(1);
    }
}

