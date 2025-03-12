#include "sd_writer.hpp"
#include <TimeLib.h>

// External buffer declared in main.cpp
extern uint8_t sdWriterBuffer[];

namespace baja {
namespace storage {

SDWriter::SDWriter(buffer::RingBuffer<data::ChannelSample, adc::RING_BUFFER_SIZE>& ringBuffer)
    : ringBuffer_(ringBuffer),
      writeBufferPos_(0),
      fileCreationTime_(0),
      bytesWritten_(0),
      lastError_(0),
      healthy_(true),
      lastErrorTime_(0),
      consecutiveErrors_(0),
      totalErrors_(0),
      lastSuccessfulWrite_(0),
      lastFlushTime_(0),
      totalFlushes_(0),
      maxFlushTime_(0) {
}

SDWriter::~SDWriter() {
    // Close any open file
    closeFile();
}

bool SDWriter::begin(uint8_t chipSelect) {
    Serial.println("SD: Initializing");
    
    // Initialize SD card in SDIO mode
    if (!sd_.begin(SdioConfig(FIFO_SDIO))) {
        // If SDIO fails, try SPI mode as fallback
        if (!sd_.begin(SdSpiConfig(chipSelect, SHARED_SPI, SD_SCK_MHZ(50)))) {
            recordError(-1, "SD: Both SDIO and SPI initialization failed");
            return false;
        }
        Serial.println("SD: SPI mode initialized");
    } else {
        Serial.println("SD: SDIO mode initialized");
    }
    
    // Print SD card info (only essential info, reduce verbosity)
    uint32_t cardSize = sd_.card()->sectorCount();
    if (cardSize) {
        float cardSizeGB = 0.000512 * cardSize;
        Serial.print("SD: Card size: ");
        Serial.print(cardSizeGB);
        Serial.println(" GB");
    }
    
    Serial.print("SD: Volume is FAT");
    Serial.println(int(sd_.fatType()));
    
    uint32_t freeKB = sd_.vol()->freeClusterCount();
    freeKB *= sd_.vol()->sectorsPerCluster() / 2;
    Serial.print("SD: Free space: ");
    Serial.print(freeKB / 1024.0);
    Serial.println(" GB");
    
    // Test write speed with a smaller test (less verbose)
    uint8_t testBuffer[4096];
    memset(testBuffer, 0xAA, sizeof(testBuffer));
    
    FsFile testFile;
    if (testFile.open("sdtest.bin", O_RDWR | O_CREAT | O_TRUNC)) {
        uint32_t start = micros();
        for (int i = 0; i < 25; i++) {
            testFile.write(testBuffer, sizeof(testBuffer));
        }
        testFile.flush();
        uint32_t end = micros();
        testFile.close();
        
        float writeTime = (end - start) / 1000000.0;
        float writeSpeed = (25 * 4) / writeTime;
        
        Serial.print("SD: Write speed: ");
        Serial.print(writeSpeed);
        Serial.println(" KB/s");
        
        // Delete test file
        sd_.remove("sdtest.bin");
    } else {
        recordError(-2, "SD: Could not create test file");
    }
    
    healthy_ = true;
    lastSuccessfulWrite_ = millis();
    
    return true;
}

void SDWriter::setChannelNames(const std::vector<adc::ChannelConfig>& channelConfigs) {
    channelNames_.clear();
    for (const auto& config : channelConfigs) {
        if (config.enabled) {
            channelNames_.push_back(config.name);
        }
    }
    
    Serial.print("SD: Set ");
    Serial.print(channelNames_.size());
    Serial.println(" channel names");
}

size_t SDWriter::process() {
    // Check for file health
    if (!healthy_) {
        // Try to recover if unhealthy for more than 10 seconds
        if (millis() - lastErrorTime_ > 10000) {
            Serial.println("SD: Attempting recovery after errors");
            closeFile();
            if (!createNewFile()) {
                // Still failing, but don't keep printing errors
                return 0;
            } else {
                // Recovered successfully
                healthy_ = true;
                Serial.println("SD: Recovery successful");
            }
        } else {
            // Still in cooldown period, don't retry yet
            return 0;
        }
    }
    
    // Check if we need to rotate the file
    if (shouldRotateFile()) {
        closeFile();
        if (!createNewFile()) {
            return 0;
        }
    }
    
    // Check if file is open
    if (!dataFile_.isOpen()) {
        if (!createNewFile()) {
            return 0;
        }
    }
    
    // Read samples from the ring buffer
    size_t availableSamples = ringBuffer_.available();
    
    if (availableSamples == 0) {
        // No samples available, but still flush if buffer has been waiting too long
        if (writeBufferPos_ > 0 && millis() - lastFlushTime_ > 5000) {
            flushBuffer();
        }
        return 0;
    }
    
    // Determine batch size based on available samples and buffer capacity
    size_t remainingBufferSpace = MAX_BUFFER_SIZE - writeBufferPos_;
    size_t estimatedSampleSize = 40; // Average CSV line length estimate
    size_t maxSamplesForBuffer = remainingBufferSpace / estimatedSampleSize;
    
    // Limit batch size to what's available
    size_t samplesInBatch = min(availableSamples, maxSamplesForBuffer);
    
    // Read only 1000 samples at a time at most, to avoid long blocking
    samplesInBatch = min(samplesInBatch, 1000UL);
    
    // Read the batch of samples
    data::ChannelSample* sampleBatch = new data::ChannelSample[samplesInBatch];
    size_t samplesRead = ringBuffer_.readMultiple(sampleBatch, samplesInBatch);
    
    // Only log large batches to reduce noise
    if (samplesRead > 1000) {
        Serial.print("SD: Processing ");
        Serial.print(samplesRead);
        Serial.println(" samples");
    }
    
    // Process each sample
    size_t samplesWritten = 0;
    for (size_t i = 0; i < samplesRead; i++) {
        // Format as CSV
        std::string line = sampleBatch[i].toCSV();
        
        // Add channel name if available
        if (sampleBatch[i].channelIndex < channelNames_.size()) {
            line += "," + channelNames_[sampleBatch[i].channelIndex];
        }
        
        // Add newline
        line += "\n";
        
        // Add to the write buffer
        if (addLineToBuffer(line)) {
            samplesWritten++;
        } else {
            // If buffer is full at this point, that's a problem
            recordError(-3, "SD: Buffer overflow while adding line");
            break;
        }
    }
    
    // Clean up
    delete[] sampleBatch;
    
    // Check if buffer should be flushed
    if (writeBufferPos_ >= BUFFER_FLUSH_THRESHOLD || 
        (writeBufferPos_ >= MIN_WRITE_SIZE && millis() - lastFlushTime_ > 1000)) {
        flushBuffer();
    }
    
    return samplesWritten;
}

bool SDWriter::createNewFile(bool addHeader) {
    // Close any open file
    closeFile();
    
    // Generate a filename
    currentFilename_ = generateFilename();
    
    Serial.print("SD: Creating new file: ");
    Serial.println(currentFilename_.c_str());
    
    // Create the file
    if (!dataFile_.open(currentFilename_.c_str(), O_WRONLY | O_CREAT | O_TRUNC)) {
        recordError(-4, "SD: Failed to create file");
        return false;
    }
    
    // Reset counters
    fileCreationTime_ = millis();
    bytesWritten_ = 0;
    writeBufferPos_ = 0;
    lastFlushTime_ = millis();
    
    // Write header if requested
    if (addHeader) {
        if (!writeHeader()) {
            recordError(-5, "SD: Failed to write header");
            closeFile();
            return false;
        }
    }
    
    // Update status
    lastSuccessfulWrite_ = millis();
    consecutiveErrors_ = 0;
    
    return true;
}

bool SDWriter::closeFile() {
    // Flush any remaining data
    if (writeBufferPos_ > 0) {
        flushBuffer();
    }
    
    // Close the file
    if (dataFile_.isOpen()) {
        dataFile_.close();
        return true;
    }
    
    return false;
}

std::string SDWriter::getCurrentFilename() const {
    return currentFilename_;
}

size_t SDWriter::getBytesWritten() const {
    return bytesWritten_;
}

bool SDWriter::shouldRotateFile() const {
    // Check if the file has been open too long
    if (millis() - fileCreationTime_ >= data::FILE_ROTATION_INTERVAL_MS) {
        return true;
    }
    
    // Also rotate file if it gets too large (100MB)
    if (bytesWritten_ >= 100 * 1024 * 1024) {
        return true;
    }
    
    return false;
}

bool SDWriter::flush() {
    return flushBuffer();
}

int SDWriter::getLastError() const {
    return lastError_;
}

bool SDWriter::isHealthy() const {
    return healthy_;
}

void SDWriter::resetHealth() {
    healthy_ = true;
    consecutiveErrors_ = 0;
    lastErrorTime_ = 0;
}

std::string SDWriter::generateFilename() const {
    char filename[32];
    
    // Get current time
    time_t t = now();
    
    // Format as YYYYMMDD_HHMMSS.csv
    sprintf(filename, "data_%04d%02d%02d_%02d%02d%02d.csv",
            year(t), month(t), day(t),
            hour(t), minute(t), second(t));
    
    return std::string(filename);
}

bool SDWriter::writeHeader() {
    // Start with timestamp and channel columns
    std::string header = "timestamp_us,channel_index,raw_value";
    
    // Add names if available
    if (!channelNames_.empty()) {
        header += ",channel_name";
    }
    
    // Add newline
    header += "\n";
    
    // Write to buffer
    return addLineToBuffer(header);
}

bool SDWriter::flushBuffer() {
    // Check if there's anything to write
    if (writeBufferPos_ == 0) {
        return true;
    }
    
    // Don't log every flush to reduce noise
    if (writeBufferPos_ > 60000) {
        Serial.print("SD: Flushing ");
        Serial.print(writeBufferPos_);
        Serial.println(" bytes");
    }
    
    uint32_t startTime = micros();
    
    // Disable interrupts during SD write to prevent corruption
    noInterrupts();
    
    // Write the buffer to the file - Access the globally defined buffer
    size_t bytesWritten = dataFile_.write(::sdWriterBuffer, writeBufferPos_);
    
    // Re-enable interrupts
    interrupts();
    
    uint32_t flushTime = micros() - startTime;
    
    if (bytesWritten != writeBufferPos_) {
        recordError(-6, "SD: Write error during flush");
        return false;
    }
    
    // Update counters
    bytesWritten_ += bytesWritten;
    writeBufferPos_ = 0;
    lastFlushTime_ = millis();
    totalFlushes_++;
    maxFlushTime_ = max(maxFlushTime_, flushTime);
    lastSuccessfulWrite_ = millis();
    
    // Sync file occasionally to prevent data loss on power failure
    // Only sync every 10 flushes to reduce wear on the card
    if (totalFlushes_ % 10 == 0) {
        dataFile_.sync();
    }
    
    return true;
}

bool SDWriter::addLineToBuffer(const std::string& line) {
    // Check if line will fit in the buffer
    if (writeBufferPos_ + line.length() > MAX_BUFFER_SIZE) {
        // Buffer is full, flush it first
        if (!flushBuffer()) {
            return false;
        }
    }
    
    // Add line to buffer - Access the globally defined buffer
    memcpy((char*)::sdWriterBuffer + writeBufferPos_, line.c_str(), line.length());
    writeBufferPos_ += line.length();
    
    return true;
}

void SDWriter::recordError(int errorCode, const char* errorMessage) {
    lastError_ = errorCode;
    lastErrorTime_ = millis();
    consecutiveErrors_++;
    totalErrors_++;
    
    // Only log every few errors to prevent log spam
    if (consecutiveErrors_ == 1 || consecutiveErrors_ % 10 == 0) {
        Serial.print("SD ERROR #");
        Serial.print(totalErrors_);
        Serial.print(": ");
        Serial.print(errorMessage);
        Serial.print(" (code ");
        Serial.print(errorCode);
        Serial.println(")");
    }
    
    // Mark as unhealthy if too many consecutive errors
    if (consecutiveErrors_ >= 3) {
        healthy_ = false;
    }
}

} // namespace storage
} // namespace baja