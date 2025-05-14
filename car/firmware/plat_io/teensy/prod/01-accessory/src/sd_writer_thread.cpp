#include "sd_writer_thread.hpp"
#include <Arduino.h>


namespace baja {
namespace sd {

SDWriterThread::SDWriterThread(data::DataBuffer& dataBuffer, uint8_t chipSelectPin, SdSpiConfig sdConfig)
    : dataBuffer_(dataBuffer)
    , chipSelectPin_(chipSelectPin)
    , sdConfig_(sdConfig)
    , threadId_(-1)
    , running_(false)
    , fileCounter_(0)
    , bytesWritten_(0)
    , errorCount_(0)
{
    // Set default basename
    strcpy(basename_, "data");
}

SDWriterThread::~SDWriterThread() {
    stop();
    
    // Close file if open
    if (dataFile_) {
        dataFile_.close();
    }
}

int SDWriterThread::start() {
    if (isRunning()) {
        return threadId_; // Already running
    }
    
    // Initialize SD card
    if (!initializeSD()) {
        Serial.println("Failed to initialize SD card!");
        return -1;
    }
    
    // Start thread
    running_ = true;
    threadId_ = threads.addThread(threadFunction, this, 8 * 1024); // 8KB stack
    
    if (threadId_ < 0) {
        Serial.println("Failed to create SD writer thread!");
        running_ = false;
        return -1;
    }
    
    Serial.print("SD writer thread started with ID: ");
    Serial.println(threadId_);
    
    return threadId_;
}

void SDWriterThread::stop() {
    if (isRunning()) {
        running_ = false;
        
        // Wait for thread to exit
        threads.wait(threadId_, 2000); // Wait up to 2 seconds
        
        // Close the file if it's open
        if (dataFile_) {
            dataFile_.close();
        }
        
        threadId_ = -1;
        
        Serial.println("SD writer thread stopped");
    }
}

bool SDWriterThread::isRunning() const {
    return running_ && threadId_ >= 0;
}

uint64_t SDWriterThread::getBytesWritten() const {
    Threads::Scope lock(const_cast<Threads::Mutex&>(globalMutex_));
    return bytesWritten_;
}

uint32_t SDWriterThread::getErrorCount() const {
    Threads::Scope lock(const_cast<Threads::Mutex&>(globalMutex_));
    return errorCount_;
}

void SDWriterThread::setBasename(const char* basename) {
    // Use a local mutex instead of the global one
    Threads::Scope lock(globalMutex_);
    
    // Copy the basename, ensuring it's not too long
    strncpy(basename_, basename, sizeof(basename_) - 1);
    basename_[sizeof(basename_) - 1] = '\0'; // Ensure null termination
}

void SDWriterThread::rotateFile() {
    // Use a local mutex instead of the global one
    Threads::Scope lock(globalMutex_);
    
    // Close current file if open
    if (dataFile_) {
        dataFile_.close();
    }
    
    // Force creation of a new file on next write
    fileCounter_++;
    
    // New file will be created on the next write cycle
}

void SDWriterThread::threadFunction(void* arg) {
    SDWriterThread* self = static_cast<SDWriterThread*>(arg);
    
    Serial.println("SD writer thread function started");
    
    // Create the first data file
    if (!self->createNewFile()) {
        Serial.println("Failed to create initial data file!");
        self->errorCount_++;
    }
    
    // Main loop - check for data to write periodically
    constexpr unsigned long CHECK_INTERVAL_MS = 100; // Check every 100ms
    unsigned long lastCheckTime = millis();
    
    while (self->running_) {
        unsigned long currentTime = millis();
        
        // Check if it's time to look for data to write
        if (currentTime - lastCheckTime >= CHECK_INTERVAL_MS) {
            lastCheckTime = currentTime;
            
            // Only process if the data buffer has triggered a write
            if (self->dataBuffer_.shouldTriggerSDWrite()) {
                // Get a segment to write
                auto [samples, count] = self->dataBuffer_.beginSDWrite();
                
                if (samples != nullptr && count > 0) {
                    // Write the samples to SD
                    bool success = self->writeToSD(samples, count);
                    
                    // Complete the write operation
                    self->dataBuffer_.completeSDWrite(success);
                    
                    if (!success) {
                        self->errorCount_++;
                        
                        // Print error info every 10 errors to avoid flooding serial
                        if (self->errorCount_ % 10 == 1) {
                            Serial.print("SD write error #");
                            Serial.println(self->errorCount_);
                        }
                    }
                }
            }
        }
        
        // Yield to other threads
        threads.yield();
        delay(1); // Small delay to prevent tight loop
    }
    
    // Close file before exiting
    if (self->dataFile_) {
        self->dataFile_.close();
    }
    
    Serial.println("SD writer thread function exiting");
}

bool SDWriterThread::initializeSD() {
    Serial.println("Initializing SD card...");
    
    // For Teensy 4.1, try SDIO mode first which is much faster
    if (!sd_.begin(SdioConfig(FIFO_SDIO))) {
        Serial.println("SDIO initialization failed, trying SPI mode...");
        if (!sd_.begin(sdConfig_)) {
            Serial.println("SD card initialization failed!");
            return false;
        }
    }
    
    Serial.println("SD card initialized successfully");
    
    // Check for available space
    uint64_t freeSpace = sd_.freeClusterCount();
    freeSpace *= sd_.bytesPerCluster();
    
    Serial.print("SD card free space: ");
    Serial.print(freeSpace / (1024 * 1024));
    Serial.println(" MB");
    
    // Print FAT type
    Serial.print("SD card file system type: ");
    switch (sd_.fatType()) {
        case FAT_TYPE_FAT12: Serial.println("FAT12"); break;
        case FAT_TYPE_FAT16: Serial.println("FAT16"); break;
        case FAT_TYPE_FAT32: Serial.println("FAT32"); break;
        case FAT_TYPE_EXFAT: Serial.println("exFAT"); break;
        default: Serial.println("unknown"); break;
    }
    
    return true;
}

bool SDWriterThread::createNewFile() {
    // Close current file if open
    if (dataFile_) {
        dataFile_.close();
    }
    
    // Generate filename with timestamp
    char filename[128];
    
    // Get current time from Teensy
    unsigned long currentTime = millis();
    
#if SD_WRITER_USE_CSV_FORMAT
    // For CSV format, use .csv extension
    // Format: basename_YYYYMMDD_HHMMSS_counter.csv
    sprintf(filename, "%s_%010lu_%03lu.csv", 
            basename_, 
            currentTime, 
            fileCounter_);
#else
    // For binary format, use .bin extension
    // Format: basename_YYYYMMDD_HHMMSS_counter.bin
    sprintf(filename, "%s_%010lu_%03lu.bin", 
            basename_, 
            currentTime, 
            fileCounter_);
#endif
    
    Serial.print("Creating new data file: ");
    Serial.println(filename);
    
    // Create the file
    dataFile_ = sd_.open(filename, FILE_WRITE | O_CREAT | O_TRUNC);
    
    if (!dataFile_) {
        Serial.println("Failed to create data file!");
        return false;
    }
    
    // Write header
    return writeFileHeader();
}

bool SDWriterThread::writeFileHeader() {
    if (!dataFile_) {
        return false;
    }
    
#if SD_WRITER_USE_CSV_FORMAT
    // Write CSV header
    dataFile_.println("timestamp,channel_index,channel_name,raw_value,voltage");
    
    // Flush to ensure header is written
    dataFile_.flush();
    
    // Update bytes written counter
    Threads::Scope lock(globalMutex_);
    bytesWritten_ += 40; // Approximate header size
    
    return true;
#else
    // Create a simple binary header structure
    struct FileHeader {
        char magic[4];          // Magic identifier "BAJA"
        uint32_t version;       // File format version
        uint64_t timestamp;     // File creation time
        uint8_t numChannels;    // Number of channels
        uint32_t sampleSize;    // Size of each sample in bytes
    };
    
    // Initialize header with data
    FileHeader header;
    memcpy(header.magic, "BAJA", 4);
    header.version = 1;
    header.timestamp = millis();
    header.numChannels = static_cast<uint8_t>(baja::adc::ChannelName::NUM_CHANNELS);
    header.sampleSize = sizeof(data::ChannelSample);
    
    // Write header to file
    size_t bytesWritten = dataFile_.write(&header, sizeof(header));
    
    // Also write channel configuration information
    for (int i = 0; i < static_cast<int>(baja::adc::ChannelName::NUM_CHANNELS); i++) {
        // Write channel index and description
        dataFile_.write(&i, sizeof(i));
        
        // Create channel description string
        char description[32];
        sprintf(description, "Channel_%d", i);
        dataFile_.write(description, sizeof(description));
    }
    
    // Flush to ensure header is written
    dataFile_.flush();
    
    // Update bytes written counter
    Threads::Scope lock(globalMutex_);
    bytesWritten_ += bytesWritten;
    
    return bytesWritten == sizeof(header);
#endif
}

bool SDWriterThread::writeToSD(const data::ChannelSample* samples, size_t count) {
  if (!dataFile_ || !samples || count == 0) {
      return false;
  }
  
  // Check if we need to rotate file (for example, max file size reached)
  constexpr size_t MAX_FILE_SIZE = 100 * 1024 * 1024; // 100MB
  
#if SD_WRITER_USE_CSV_FORMAT
  // For CSV format, estimate file size growth differently
  // Approximately 50 bytes per sample in CSV format (timestamp,channel,value,voltage)
  constexpr size_t CSV_BYTES_PER_SAMPLE = 50;
  if (dataFile_.size() + (count * CSV_BYTES_PER_SAMPLE) > MAX_FILE_SIZE) {
      if (!createNewFile()) {
          Serial.println("Failed to create new file for rotation!");
          return false;
      }
  }
  
  // Use a buffer for CSV writing to improve performance
  constexpr size_t CSV_BUFFER_SIZE = 4096;
  char csvBuffer[CSV_BUFFER_SIZE];
  size_t bufferPos = 0;
  size_t totalBytesWritten = 0;
  
  // Process each sample
  for (size_t i = 0; i < count; i++) {
      // Format sample as CSV: timestamp,channel_index,raw_value,voltage
      double voltage = ((double)(samples[i].rawValue) * 5.0) / (1 << 24);
      int bytesWritten = snprintf(csvBuffer + bufferPos, 
                                  CSV_BUFFER_SIZE - bufferPos,
                                  "%llu,%d,%lu,%.6f\n",
                                  samples[i].timestamp,
                                  samples[i].channelIndex,
                                  samples[i].rawValue,
                                  voltage);
                                  
      if (bytesWritten < 0 || bytesWritten >= (CSV_BUFFER_SIZE - bufferPos)) {
          // Buffer full or error, flush it now
          size_t written = dataFile_.write(csvBuffer, bufferPos);
          totalBytesWritten += written;
          
          if (written != bufferPos) {
              Serial.println("Error writing CSV data to SD!");
              return false;
          }
          
          // Reset buffer and retry this sample
          bufferPos = 0;
          i--; // Retry this sample
          continue;
      }
      
      bufferPos += bytesWritten;
      
      // If buffer is getting full, flush it
      if (bufferPos > (CSV_BUFFER_SIZE - 100)) {
          size_t written = dataFile_.write(csvBuffer, bufferPos);
          totalBytesWritten += written;
          
          if (written != bufferPos) {
              Serial.println("Error writing CSV data to SD!");
              return false;
          }
          
          bufferPos = 0;
      }
  }
  
  // Flush any remaining data in buffer
  if (bufferPos > 0) {
      size_t written = dataFile_.write(csvBuffer, bufferPos);
      totalBytesWritten += written;
      
      if (written != bufferPos) {
          Serial.println("Error writing final CSV data to SD!");
          return false;
      }
  }
  
  // Update bytes written counter
  Threads::Scope lock(globalMutex_);
  bytesWritten_ += totalBytesWritten;
  
#else
  // Binary format - check file size with exact binary size
  size_t dataSize = count * sizeof(data::ChannelSample);
  
  if (dataFile_.size() + dataSize > MAX_FILE_SIZE) {
      if (!createNewFile()) {
          Serial.println("Failed to create new file for rotation!");
          return false;
      }
  }
  
  // For optimal performance with large blocks, use a
  // large buffer size such as 4KB or 32KB blocks
  constexpr size_t OPTIMAL_BLOCK_SIZE = 32 * 1024; // 32KB is often optimal
  
  // If data is larger than optimal block size, write in chunks
  if (dataSize > OPTIMAL_BLOCK_SIZE && count > 1) {
      const size_t samplesPerBlock = OPTIMAL_BLOCK_SIZE / sizeof(data::ChannelSample);
      size_t remainingSamples = count;
      size_t bytesWritten = 0;
      size_t sampleOffset = 0;
      
      // Write in optimal-sized blocks
      while (remainingSamples > 0) {
          const size_t blockSamples = (remainingSamples > samplesPerBlock) ? 
                                     samplesPerBlock : remainingSamples;
          const size_t blockSize = blockSamples * sizeof(data::ChannelSample);
          
          size_t written = dataFile_.write(&samples[sampleOffset], blockSize);
          
          if (written != blockSize) {
              Serial.println("Error writing block to SD!");
              return false;
          }
          
          bytesWritten += written;
          sampleOffset += blockSamples;
          remainingSamples -= blockSamples;
      }
      
      // Update bytes written counter
      Threads::Scope lock(globalMutex_);
      bytesWritten_ += bytesWritten;
  } else {
      // For smaller data, write all at once
      size_t bytesWritten = dataFile_.write(samples, dataSize);
      
      // Update bytes written counter
      Threads::Scope lock(globalMutex_);
      bytesWritten_ += bytesWritten;
      
      // Check if all data was written
      if (bytesWritten != dataSize) {
          return false;
      }
  }
#endif
  
  // Flush to SD card
  // Note: Flushing less frequently can improve performance,
  // but increases risk of data loss on power failure
  dataFile_.flush();
  
  return true;
}


} // namespace sd
} // namespace baja