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
const char* HTTP_SERVER_URL = "http://192.168.137.84:9365/";



// HTTP client thread function
void httpClientThreadFunc() {
    baja::util::Debug::info("HTTP client thread started");
    
    // Configure request
    httpRequest.setTimeout(10); // 10 seconds timeout
    httpRequest.setDebug(true); // Enable debugging
    
    // Set up callback for response handling
    httpRequest.onReadyStateChange([](void* optParm, AsyncHTTPRequest* request, int readyState) {
        if (readyState == readyStateDone) {
            baja::util::Debug::info("HTTP response received");
            baja::util::Debug::info("Response code: " + String(request->responseHTTPcode()));
            
            if (request->responseHTTPcode() == 200) {
                String responseText = request->responseText();
                if (responseText.length() > 0) {
                    // Log a snippet of the response if it's long
                    if (responseText.length() > 100) {
                        baja::util::Debug::info("Response: " + responseText.substring(0, 100) + "...");
                    } else {
                        baja::util::Debug::info("Response: " + responseText);
                    }
                }
            }
        }
    }, nullptr);
    
    // Variables for timing and statistics
    uint32_t lastRequestTime = 0;
    uint32_t totalSamplesSent = 0;
    uint32_t batchesSent = 0;
    
    // Main loop
    while (true) {
        uint32_t currentTime = millis();
        
        // Check if it's time to send a new request
        if (currentTime - lastRequestTime >= HTTP_REQUEST_INTERVAL) {
            // Only proceed if the previous request is done
            if (httpRequest.readyState() == readyStateUnsent || httpRequest.readyState() == readyStateDone) {
                // Get data from the ring buffer
                size_t availableSamples = sampleBuffer.available();
                
                if (availableSamples > 0) {
                    // Limit batch size to keep requests manageable
                    size_t samplesToSend = min(availableSamples, (size_t)10);
                    
                    // Build JSON data string directly
                    String jsonData = "{\"device\":\"Teensy41\",";
                    jsonData += "\"timestamp\":" + String(currentTime) + ",";
                    jsonData += "\"samples\":[";
                    
                    // Add each sample to the JSON string
                    size_t samplesAdded = 0;
                    for (size_t i = 0; i < samplesToSend; i++) {
                        baja::data::ChannelSample sample;
                        if (sampleBuffer.peek(sample, i)) {
                            // Add comma if not the first sample
                            if (samplesAdded > 0) {
                                jsonData += ",";
                            }
                            
                            // Add sample data
                            jsonData += "{\"ts\":" + String(sample.timestamp) + ",";
                            jsonData += "\"ch\":" + String(sample.channelIndex) + ",";
                            jsonData += "\"val\":" + String(sample.rawValue);
                            
                            // Add channel name if valid and available
                            if (channelConfigsArray && sample.channelIndex < baja::adc::ADC_CHANNEL_COUNT) {
                                const std::string& name = channelConfigsArray[sample.channelIndex].name;
                                if (!name.empty()) {
                                    jsonData += ",\"name\":\"" + String(name.c_str()) + "\"";
                                }
                            }
                            
                            jsonData += "}";
                            samplesAdded++;
                        }
                    }
                    
                    // Close JSON array and object
                    jsonData += "]}";
                    
                    baja::util::Debug::detail("Sending HTTP request with " + String(samplesAdded) + " samples");
                    baja::util::Debug::detail("JSON data: " + jsonData);
                    
                    // Send the HTTP request exactly like the working example
                    bool requestOpenResult = httpRequest.open("POST", HTTP_SERVER_URL);
                    
                    if (requestOpenResult) {
                        bool sendResult = httpRequest.send(jsonData);
                        
                        if (sendResult) {
                            baja::util::Debug::info("HTTP request sent successfully");
                            
                            // Update statistics
                            lastRequestTime = currentTime;
                            totalSamplesSent += samplesAdded;
                            batchesSent++;
                            
                            // Log periodic status
                            if (batchesSent % 10 == 0) {
                                baja::util::Debug::info("Stats: " + String(batchesSent) + 
                                                     " batches, " + String(totalSamplesSent) + 
                                                     " samples sent");
                            }
                        } else {
                            baja::util::Debug::error("Failed to send HTTP request");
                        }
                    } else {
                        baja::util::Debug::error("Failed to open HTTP request");
                    }
                } else {
                    baja::util::Debug::detail("No samples available to send");
                }
            } else {
                baja::util::Debug::detail("Previous request still in progress");
            }
        }
        
        // Check network status periodically
        static uint32_t lastNetworkCheckTime = 0;
        if (currentTime - lastNetworkCheckTime >= 30000) { // Check every 30 seconds
            lastNetworkCheckTime = currentTime;
            
            if (Ethernet.linkStatus() != LinkStatus_kLinkStatusUp) {
                baja::util::Debug::warning("Network link down, attempting to reconnect...");
                
                // Reinitialize Ethernet
                Ethernet.begin();
                
                // Wait briefly for reconnection
                delay(1000);
                
                // Log new status
                if (Ethernet.linkStatus() == LinkStatus_kLinkStatusUp) {
                    baja::util::Debug::info("Network link restored");
                } else {
                    baja::util::Debug::warning("Network reconnection failed, will retry later");
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

    // if (adcInitialized) {
    //     baja::util::Debug::info("Starting HTTP client thread...");
        
    //     // Initialize QNEthernet first
    //     #if USING_DHCP
    //         // Start the Ethernet connection using DHCP
    //         baja::util::Debug::info("Initializing Ethernet using DHCP...");
    //         Ethernet.begin();
    //     #else
    //         // Start the Ethernet connection using static IP
    //         baja::util::Debug::info("Initializing Ethernet using static IP...");
    //         Ethernet.begin(myIP, myNetmask, myGW);
    //         Ethernet.setDNSServerIP(mydnsServer);
    //     #endif
        
    //     // Wait for Ethernet initialization (with timeout)
    //     uint32_t ethStartTime = millis();
    //     bool ethInitSuccess = false;
        
    //     while (millis() - ethStartTime < 5000) {
    //         if (Ethernet.linkStatus() == LinkStatus_kLinkStatusUp) {
    //             ethInitSuccess = true;
    //             break;
    //         }
    //         delay(100);
    //     }
        
    //     if (ethInitSuccess) {
    //         baja::util::Debug::info("Ethernet initialized successfully");
    //         baja::util::Debug::info("IP Address: ");
    //         Serial.println(Ethernet.localIP());
            
    //         // Create HTTP client thread
    //         int httpClientThreadId = threads.addThread(httpClientThreadFunc, 0, 8192);
            
    //         if (httpClientThreadId < 0) {
    //             baja::util::Debug::error("Failed to create HTTP client thread!");
    //         } else {
    //             baja::util::Debug::info("HTTP client thread created with ID: " + String(httpClientThreadId));
    //         }
    //     } else {
    //         baja::util::Debug::warning("Ethernet initialization timed out");
    //     }
    // }
    
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