#include "adc_thread.hpp"
#include <Arduino.h>

namespace baja {
namespace adc {

// Initialize static members
volatile bool AdcThread::dataReady_ = false;
AdcThread* AdcThread::instance_ = nullptr;

AdcThread::AdcThread(SPIClass* spiHost, int8_t csPin, int8_t drdyPin, AdcDataQueue& dataQueue)
  : spiHost_(spiHost)
  , csPin_(csPin)
  , drdyPin_(drdyPin)
  , dataQueue_(dataQueue)
  , threadId_(-1)
  , running_(false)
{
  // Store instance for interrupt handler
  instance_ = this;
  
  // Initialize pins
  pinMode(csPin_, OUTPUT);
  digitalWrite(csPin_, HIGH);  // CS is active low, so set it high initially
  
  pinMode(drdyPin_, INPUT_PULLUP);
}

AdcThread::~AdcThread() {
  stop();
  instance_ = nullptr;
}

int AdcThread::start() {
  if (isRunning()) {
      Serial.println("ADC thread already running");
      return threadId_; // Already running
  }
  
  // Initialize ADC
  Serial.println("Starting ADC initialization...");
  if (!initializeAdc()) {
      Serial.println("Failed to initialize ADC!");
      return -1;
  }
  
  // Start thread
  running_ = true;
  Serial.println("Starting ADC thread...");
  
  // Create the thread with a 8KB stack size
  threadId_ = threads.addThread(threadFunction, this, 8*1024); // 8KB stack
  
  if (threadId_ < 0) {
      Serial.println("Failed to create ADC thread!");
      running_ = false;
      return -1;
  }
  
  Serial.print("ADC thread started with ID: ");
  Serial.println(threadId_);
  
  return threadId_;
}

void AdcThread::stop() {
  if (isRunning()) {
      running_ = false;
      
      // Wait for thread to exit
      threads.wait(threadId_, 2000); // Wait up to 2 seconds
      
      // Detach interrupt if it exists
      if (digitalPinToInterrupt(drdyPin_) >= 0) {
          detachInterrupt(digitalPinToInterrupt(drdyPin_));
      }
      
      // Put ADC in standby mode to save power
      adc_.setADCMode(STANDBY);
      
      threadId_ = -1;
      
      Serial.println("ADC thread stopped");
  }
}

bool AdcThread::isRunning() const {
  return running_ && threadId_ >= 0;
}

void AdcThread::threadFunction(void* arg) {
  AdcThread* self = static_cast<AdcThread*>(arg);
  
  Serial.println("ADC thread function started");
  
  // Setup faster SPI for data acquisition phase
  self->spiHost_->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
  self->spiHost_->endTransaction();
  
  // Instead of using polling, let's use the DRDY interrupt for maximum speed
  // Set up DRDY interrupt
  attachInterrupt(digitalPinToInterrupt(self->drdyPin_), drdyInterruptHandler, FALLING);
  
  // Initialize variables for ADC reading
  ad717x_data_t data;
  AdcSample sample;
  
  // Start continuous conversion mode
  int32_t result = self->adc_.setADCMode(CONTINUOUS);
  if (result != 0) {
      Serial.println("Failed to set ADC to continuous mode!");
      self->running_ = false;
      return;
  }
  
  Serial.println("ADC set to continuous mode, entering main loop");
  
  // Keep track of timing
  unsigned long lastStatusTime = millis();
  unsigned long sampleCount = 0;
  unsigned long consecutiveErrors = 0;
  
  // Data collection variables
  int dataCount = 0;
  uint32_t dataValues[static_cast<int>(ChannelName::NUM_CHANNELS)];
  uint32_t simulated_value = 0; // Added: simulated value counter///////////////////////////////////////////////////////////////////////////////////////////////
  
  // Main loop using interrupt-driven approach
  while (self->running_) {
      // Wait for DRDY interrupt
      if (dataReady_) {
          // Reset flag immediately
          dataReady_ = false;
          
          // Read data
          result = self->adc_.contConvReadData(&data);
          
          if (result == 0) {
              // Success - store the value
              sampleCount++;
              consecutiveErrors = 0; // Reset error counter
              simulated_value++;///////////////////////////////////////////////////////////////////////////////////////////////
              
              // Store data in the channel array
              int channelIndex = data.status.active_channel;
              if (channelIndex < static_cast<int>(ChannelName::NUM_CHANNELS)) {
                  dataValues[channelIndex] = data.value;
                  dataCount++;

                  
                  // Push sample to the queue
                  sample.timestamp = micros();
                  sample.channelName = getChannelNameByIndex(channelIndex);
                  sample.rawValue = simulated_value;///////////////////////////////////////////////////////////////////////////////////////////////
                //   sample.rawValue = data.value;
                  
                  bool pushed = self->dataQueue_.push(sample);
                  if (!pushed) {
                      // Queue is full - this shouldn't happen with proper queue size
                      if (millis() - lastStatusTime >= 5000) { // Only log every 5 seconds
                          Serial.println("Queue full, dropping samples");
                          lastStatusTime = millis();
                      }
                  }
                  
                  // When we have collected data from all channels, print debug info
                  if (dataCount % 100 == 0) { // Only print every 100 samples to avoid slowing down
                      // Print out voltage values - do this less frequently
                      if (millis() - lastStatusTime >= 1000) {
                          Serial.print("ADC values: ");
                          for (int i = 0; i < static_cast<int>(ChannelName::NUM_CHANNELS); i++) {
                              // Convert raw value to voltage (5V reference, 24-bit)
                              float voltage = ((double)(dataValues[i]) * 5.0) / (1 << 24);
                              Serial.print(voltage, 3);
                              Serial.print("V ");
                          }
                          Serial.println();
                          
                          Serial.print("Sample rate: ");
                          Serial.print(sampleCount);
                          Serial.println(" samples/sec");
                          
                          sampleCount = 0;
                          lastStatusTime = millis();
                      }
                  }
              }
          } else {
              // Error reading data
              consecutiveErrors++;
              
              if (consecutiveErrors % 10 == 0) { // Only log every 10 errors
                  Serial.print("ADC read error: ");
                  Serial.println(result);
              }
              
              // Reset if too many consecutive errors
              if (consecutiveErrors > 100) {
                  Serial.println("Too many consecutive errors, resetting ADC");
                  self->resetAdc();
                  consecutiveErrors = 0;
                  delay(100); // Give it some time to recover
              }
          }
      } else {
          // No data ready, yield briefly to other threads
          threads.yield();
          
          // Check if we haven't received data for a long time
          static unsigned long lastDataTime = millis();
          if (millis() - lastDataTime > 10000) { // 1 second timeout
              Serial.println("No ADC data for 1 second, resetting");
              self->resetAdc();
              lastDataTime = millis();
          }
          
          // Update last data time if we've received data
          if (sampleCount > 0) {
              lastDataTime = millis();
          }
      }
  }
  
  Serial.println("ADC thread function ending");
}



bool AdcThread::initializeAdc() {
  const AdcConfig& config = getAdcConfig();
  
  // Set up SPI with the right settings
  spiHost_->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  spiHost_->endTransaction();
  
  // Prepare initialization parameter structure - simpler approach
  ad717x_init_param_t initParam;
  initParam.active_device = ID_AD7175_8;
  initParam.mode = CONTINUOUS;
  initParam.ref_en = true;
  initParam.stat_on_read_en = true;
  
  // Set up channels - using reduced number of channels (6)
  initParam.chan_map.resize(static_cast<int>(ChannelName::NUM_CHANNELS));
  
  // Just a single setup for all channels (simpler approach)
  initParam.setups.resize(1);
  
  // Configure the filter for setup 0
  initParam.setups[0].filter_config.sinc3_map = config.sinc3Map;
  initParam.setups[0].filter_config.enhfilten = config.enhFilterEnabled;
  initParam.setups[0].filter_config.enhfilt = config.enhFilter;
  initParam.setups[0].filter_config.oder = config.filterOrder;
  initParam.setups[0].filter_config.odr = config.outputDataRate;
  
  // Configure channel setup parameters
  initParam.setups[0].setup.bi_polar = false;
  initParam.setups[0].setup.ref_source = AVDD_AVSS;
  initParam.setups[0].setup.input_buff = false;
  initParam.setups[0].setup.ref_buff = false;
  initParam.setups[0].gain = 1.0;
  
  // Configure channels using the DB44 pin mappings
  for (int i = 0; i < static_cast<int>(ChannelName::NUM_CHANNELS); i++) {
      const ChannelConfig& channelConfig = config.channels[i];
      
      if (channelConfig.enabled) {
          // Set up channel map - all channels use setup 0
          initParam.chan_map[i].channel_enable = true;
          initParam.chan_map[i].setup_sel = 0;
          initParam.chan_map[i].inputs = channelConfig.inputPins;
      } else {
          // Disabled channel
          initParam.chan_map[i].channel_enable = false;
      }
  }
  
  // Initialize ADC - using a try-catch block in case of SPI errors
  Serial.println("Initializing ADC hardware...");
  int32_t result = -1;
  
  // We'll try the initialization multiple times with delays in between
  for (int attempt = 0; attempt < 3; attempt++) {
    // Reset the ADC before initialization
    adc_.reset();
    delay(100); // Allow time for reset
    
    result = adc_.init(initParam, spiHost_, csPin_);
    if (result == 0) {
        Serial.println("ADC initialized successfully!");
        
        // Configure for continuous conversion
        result = adc_.setADCMode(CONTINUOUS);
        if (result == 0) {
            Serial.println("ADC set to continuous mode");
            return true;
        } else {
            Serial.print("Failed to set continuous mode, error: ");
            Serial.println(result);
        }
    } else {
        Serial.print("ADC init failed, attempt ");
        Serial.print(attempt + 1);
        Serial.print(", error code: ");
        Serial.println(result);
    }
    delay(250); // Wait before trying again
  }
  
  return false;
}

bool AdcThread::resetAdc() {
  Serial.println("Resetting ADC...");
  
  // First try a soft reset
  int32_t result = adc_.reset();
  
  if (result != 0) {
      Serial.println("Soft reset failed, attempting full re-initialization");
      // If soft reset fails, try to reinitialize completely
      return initializeAdc();
  }
  
  Serial.println("ADC soft reset successful");
  return true;
}

void AdcThread::handleAdcError(int32_t errorCode) {
  Serial.print("ADC Error: ");
  Serial.println(errorCode);
  
  // Try to read the status register to get more info
  int32_t readResult = adc_.readRegister(AD717X_STATUS_REG);
  if (readResult == 0) {
      Serial.print("Status register: 0x");
      Serial.println(readResult, HEX);
      
      // Check for specific errors
      if (readResult & AD717X_STATUS_REG_ADC_ERR) {
          Serial.println("ADC error bit set");
      }
      if (readResult & AD717X_STATUS_REG_CRC_ERR) {
          Serial.println("CRC error bit set");
      }
      if (readResult & AD717X_STATUS_REG_REG_ERR) {
          Serial.println("Register error bit set");
      }
  }
  
  // Try to reset the ADC
  resetAdc();
}

void AdcThread::drdyInterruptHandler() {
  if (instance_ && instance_->running_) {
      dataReady_ = true;
  }
}





} // namespace adc
} // namespace baja