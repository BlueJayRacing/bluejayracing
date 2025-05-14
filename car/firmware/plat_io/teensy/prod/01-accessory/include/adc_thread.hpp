#pragma once

#include <Arduino.h>
#include <TeensyThreads.h>
#include "ad717x.hpp"
#include "adc_channel_config.hpp"
#include "adc_data_queue.hpp"

namespace baja {
namespace adc {

/**
 * @brief ADC Thread Class
 * 
 * Manages high-priority sampling of the AD7175-8 ADC at 4kHz per channel.
 * Uses DRDY interrupt on pin 24 to synchronize readings.
 */
class AdcThread {
public:
    /**
     * @brief Construct a new Adc Thread object
     * 
     * @param spiHost SPI interface to use
     * @param csPin Chip select pin
     * @param drdyPin Data ready pin (interrupt)
     * @param dataQueue Reference to the data queue for storing samples
     */
    AdcThread(SPIClass* spiHost, int8_t csPin, int8_t drdyPin, AdcDataQueue& dataQueue);
    
    /**
     * @brief Destroy the Adc Thread object
     */
    ~AdcThread();
    
    /**
     * @brief Start the ADC thread
     * 
     * @return thread ID if successful, negative value on error
     */
    int start();
    
    /**
     * @brief Stop the ADC thread
     */
    void stop();
    
    /**
     * @brief Check if ADC thread is running
     * 
     * @return true if running, false otherwise
     */
    bool isRunning() const;

private:
    /**
     * @brief Main thread function
     */
    static void threadFunction(void* arg);
    
    /**
     * @brief Initialize the ADC
     * 
     * @return true if successful, false otherwise
     */
    bool initializeAdc();
    
    /**
     * @brief Reset and reinitialize the ADC
     * 
     * @return true if successful, false otherwise
     */
    bool resetAdc();
    
    /**
     * @brief Handle ADC error
     * 
     * @param errorCode Error code from the ADC
     */
    void handleAdcError(int32_t errorCode);
    
    /**
     * @brief Data ready interrupt handler
     */
    static void drdyInterruptHandler();

private:
    // Hardware configuration
    SPIClass* spiHost_;
    int8_t csPin_;
    int8_t drdyPin_;
    
    // Data structures
    AdcDataQueue& dataQueue_;
    AD717X adc_;
    
    // Thread control
    int threadId_;
    volatile bool running_;
    
    // Interrupt handling
    static volatile bool dataReady_;
    static AdcThread* instance_; // For interrupt handler
};

} // namespace adc
} // namespace baja