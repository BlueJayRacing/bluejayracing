#include <Arduino.h>
#include <SPI.h>
#include <TeensyThreads.h>
#include <Sdfat.h>
// #include <SD.h>
#include <TimeLib.h>

#include "adc_thread.hpp"
#include "adc_data_queue.hpp"
#include "channel_sample.hpp"
#include "data_buffer.hpp"
#include "sd_writer_thread.hpp"

// Ensure we're running on a Teensy 4.1
#if !defined(__IMXRT1062__)
#error "This code requires Teensy 4.1"
#endif

// Pin definitions
constexpr int8_t ADC_CS_PIN = 10;   // Chip select pin for AD7175-8
constexpr int8_t ADC_DRDY_PIN = 24; // Data ready pin from AD7175-8
// For Teensy 4.1 with built-in SD
#define BUILTIN_SDCARD 254
#define SD_CS_PIN BUILTIN_SDCARD

// Queue size (in samples)
constexpr size_t ADC_QUEUE_SIZE = 16384; // 16K samples for higher speed
constexpr size_t DATA_BUFFER_SIZE = 480 * 1024; // 480KB for data buffer

// Buffer for intermediate storage in external memory
EXTMEM uint8_t extBuffer[64*1024]; // 64KB transfer buffer in external memory

// Create global instances
baja::adc::AdcDataQueue adcQueue(ADC_QUEUE_SIZE);
baja::adc::AdcThread* adcThread = nullptr;
baja::data::DataBuffer* dataBuffer = nullptr;
baja::sd::SDWriterThread* sdWriterThread = nullptr;

// Thread IDs
int processingThreadId = -1;
int sdWriterThreadId = -1;

// Thread priorities (higher number is higher priority)
constexpr int ADC_THREAD_PRIORITY = 5;         // Highest priority
constexpr int PROCESSING_THREAD_PRIORITY = 4;  // High priority
constexpr int SD_WRITER_THREAD_PRIORITY = 3;   // Medium priority

// Thread stack sizes
constexpr int ADC_THREAD_STACK_SIZE = 16384;       // 16KB for ADC thread
constexpr int PROCESSING_THREAD_STACK_SIZE = 8192; // 8KB for processing thread
constexpr int SD_WRITER_STACK_SIZE = 12288;        // 12KB for SD writer thread

// Helper functions for date/time
time_t getTeensy3Time() {
    return Teensy3Clock.get();
}

void dateTime(uint16_t *date, uint16_t *time) {
    *date = FAT_DATE(year(), month(), day());
    *time = FAT_TIME(hour(), minute(), second());
}

// Processing thread function - transfers data from AdcQueue to DataBuffer
void processingThreadFunction(void* arg) {
    Serial.println("Processing thread started");
    
    baja::adc::AdcSample adcSample;
    baja::data::ChannelSample dataSample;
    
    while (true) {
        // Process samples in batches when possible
        constexpr int BATCH_SIZE = 64; // Larger batch size for higher throughput
        baja::adc::AdcSample adcSamples[BATCH_SIZE];
        baja::data::ChannelSample dataSamples[BATCH_SIZE];
        
        // Try to get a batch of samples from ADC queue
        int samplesRead = adcQueue.popBatch(adcSamples, BATCH_SIZE);
        
        if (samplesRead > 0) {
            // Convert AdcSample to ChannelSample and push to data buffer
            for (int i = 0; i < samplesRead; i++) {
                // Convert ADC sample to data sample
                dataSamples[i].timestamp = adcSamples[i].timestamp;
                dataSamples[i].channelIndex = static_cast<uint8_t>(adcSamples[i].channelName);
                dataSamples[i].rawValue = adcSamples[i].rawValue;
                
                // Example: print every 10000th sample to avoid overwhelming the console
                static int sampleCount = 0;
                if (++sampleCount % 10000 == 0) {
                    // Uncomment to debug voltage values
                    /*
                    double voltage_val = ((double)(dataSamples[i].rawValue) * 5.0) / (1 << 24);
                    Serial.print("Sample from channel: ");
                    Serial.print(dataSamples[i].channelIndex);
                    Serial.print(", Voltage: ");
                    Serial.print(voltage_val, 3);
                    Serial.print("V, Raw: ");
                    Serial.print(dataSamples[i].rawValue);
                    Serial.print(", Timestamp: ");
                    Serial.println(dataSamples[i].timestamp);
                    */
                }
            }
            
            // Push batch to data buffer
            if (dataBuffer) {
                int pushed = dataBuffer->pushBatch(dataSamples, samplesRead);
                
                // If buffer is getting full, trigger SD write
                if (dataBuffer->fillPercentage() > 0.8f) {
                    // This will be checked by SD writer thread
                    // No need to explicitly signal as it checks the buffer status
                }
                
                // Debug if not all samples were pushed
                if (pushed < samplesRead) {
                    static uint32_t lastWarningTime = 0;
                    // Only print warning occasionally to avoid flooding serial
                    if (millis() - lastWarningTime > 5000) {
                        Serial.println("WARNING: Data buffer full, dropping samples");
                        lastWarningTime = millis();
                    }
                }
            }
        } else {
            // No samples available, yield to other threads
            threads.yield();
            delayMicroseconds(10); // Very small delay to prevent tight loops
        }
    }
}
#include "test.hpp"
void setup() {
    Test test;
    test.continuousChannelReadStats();

    // Initialize serial for debugging and wait for connection
    Serial.begin(115200);
    uint32_t startTime = millis();
    while (!Serial && (millis() - startTime < 3000)); // Wait up to 3 seconds for serial
    
    Serial.println("\n\n=== Baja Data Acquisition System ===");
    Serial.println("Initializing...");
    
    // Initialize SPI with higher speed for faster sampling
    SPI.begin();
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
    SPI.endTransaction();
    
    // Set up threads system
    threads.setDefaultTimeSlice(1); // 1ms time slice for more responsive operation
    threads.setDefaultStackSize(8192); // Default 8KB stack
    
    // Initialize date/time for SD card file timestamps
    setSyncProvider(getTeensy3Time);
    SdFile::dateTimeCallback(dateTime);
    
    // Create ADC thread
    Serial.println("Initializing ADC thread...");
    adcThread = new baja::adc::AdcThread(&SPI, ADC_CS_PIN, ADC_DRDY_PIN, adcQueue);
    
    // Create data buffer
    Serial.println("Initializing data buffer...");
    dataBuffer = new baja::data::DataBuffer(DATA_BUFFER_SIZE);
    
    // Initialize SD card and writer thread
    Serial.println("Initializing SD writer thread...");
    // Use BUILTIN_SDCARD for proper SDIO access
    SdSpiConfig sdConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50));
    sdWriterThread = new baja::sd::SDWriterThread(*dataBuffer, SD_CS_PIN, sdConfig);
    
    // Start ADC thread with higher priority
    int adcThreadId = adcThread->start();
    if (adcThreadId < 0) {
        Serial.println("Failed to start ADC thread! System will operate without ADC data.");
        delete adcThread;
        adcThread = nullptr;
    } else {
        Serial.print("ADC thread started with ID: ");
        Serial.println(adcThreadId);
        
        // Set priority for ADC thread
        threads.setTimeSlice(adcThreadId, ADC_THREAD_PRIORITY);
    }
    
    // Start processing thread with specified stack size
    processingThreadId = threads.addThread(processingThreadFunction, 0, PROCESSING_THREAD_STACK_SIZE);
    
    if (processingThreadId < 0) {
        Serial.println("Failed to start processing thread!");
    } else {
        Serial.print("Processing thread started with ID: ");
        Serial.println(processingThreadId);
        
        // Set priority for processing thread - slightly lower than ADC thread
        threads.setTimeSlice(processingThreadId, PROCESSING_THREAD_PRIORITY);
    }
    
    // Start SD writer thread
    sdWriterThreadId = sdWriterThread->start();
    
    if (sdWriterThreadId < 0) {
        Serial.println("Failed to start SD writer thread! Data will not be saved.");
    } else {
        Serial.print("SD writer thread started with ID: ");
        Serial.println(sdWriterThreadId);
        
        // Set priority for SD writer thread
        threads.setTimeSlice(sdWriterThreadId, SD_WRITER_THREAD_PRIORITY);
    }
    
    Serial.println("Setup complete!");
    
    // Print available features info
    Serial.println("\nAvailable function calls:");
    Serial.println("- sdWriterThread->rotateFile() - Create a new data file");
    Serial.println("- dataBuffer->size() - Get number of samples in buffer");
    Serial.println("- dataBuffer->fillPercentage() - Get buffer fill level");
    Serial.println("- sdWriterThread->getBytesWritten() - Get bytes written to SD");
}

void loop() {
    // Main loop - monitor the system
    static unsigned long lastStatsTime = 0;
    
    // Print system stats every 5 seconds to reduce serial traffic
    if (millis() - lastStatsTime >= 5000) {
        lastStatsTime = millis();
        
        // Print ADC queue stats
        Serial.print("ADC queue: ");
        Serial.print(adcQueue.size());
        Serial.print(" / ");
        Serial.print(ADC_QUEUE_SIZE);
        Serial.print(" samples (");
        Serial.print((float)adcQueue.size() / ADC_QUEUE_SIZE * 100.0f, 1);
        Serial.println("%)");
        
        // Print data buffer stats
        if (dataBuffer) {
            Serial.print("Data buffer: ");
            Serial.print(dataBuffer->size());
            Serial.print(" / ");
            Serial.print(dataBuffer->capacity());
            Serial.print(" samples (");
            Serial.print(dataBuffer->fillPercentage() * 100.0f, 1);
            Serial.println("%)");
        }
        
        // Print SD writer stats
        if (sdWriterThread && sdWriterThread->isRunning()) {
            Serial.print("SD writer: ");
            Serial.print(sdWriterThread->getBytesWritten() / 1024);
            Serial.print(" KB written, ");
            Serial.print(sdWriterThread->getErrorCount());
            Serial.println(" errors");
        }
        
        // Check thread status
        if (processingThreadId > 0) {
            int state = threads.getState(processingThreadId);
            Serial.print("Processing thread: ");
            
            switch(state) {
                case Threads::RUNNING:
                    Serial.println("RUNNING");
                    break;
                case Threads::ENDED:
                    Serial.println("ENDED");
                    break;
                case Threads::SUSPENDED:
                    Serial.println("SUSPENDED");
                    break;
                default:
                    Serial.println(state);
            }
            
            // Check stack usage
            Serial.print("  Stack used: ");
            Serial.print(threads.getStackUsed(processingThreadId));
            Serial.print(" bytes, remaining: ");
            Serial.print(threads.getStackRemaining(processingThreadId));
            Serial.println(" bytes");
        }
        
        if (adcThread) {
            Serial.print("ADC thread: ");
            Serial.println(adcThread->isRunning() ? "RUNNING" : "STOPPED");
        }
        
        // Print a divider
        Serial.println("------------------------------");
    }
    
    // Sleep for a while - other threads will continue running
    delay(100);
}