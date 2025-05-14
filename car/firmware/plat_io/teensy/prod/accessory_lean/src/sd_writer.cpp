#include "sd_writer.hpp"
#include <TimeLib.h>

namespace baja {
namespace storage {

SDWriter::SDWriter(buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer,
                 RingBuf<FsFile, config::SD_RING_BUF_CAPACITY>* sdRingBuf)
    : dataBuffer_(ringBuffer),
      ringBuf_(sdRingBuf),
      fileCreationTime_(0),
      bytesWritten_(0),
      lastError_(0),
      healthy_(true),
      lastErrorTime_(0),
      consecutiveErrors_(0),
      totalErrors_(0),
      lastSuccessfulWrite_(0),
      lastWriteTime_(0),
      totalWrites_(0),
      maxWriteTime_(0),
      totalSamplesWritten_(0),
      wasBufferFull_(false) {
}

SDWriter::~SDWriter() {
    // Close any open file
    closeFile();
}

bool SDWriter::begin(uint8_t chipSelect) {
    util::Debug::info("SD: Initializing");
    
    // Initialize SD card in SDIO mode
    if (!sd_.begin(SdioConfig(FIFO_SDIO))) {
        // If SDIO fails, try SPI mode as fallback
        if (!sd_.begin(SdSpiConfig(chipSelect, SHARED_SPI, SD_SCK_MHZ(50)))) {
            recordError(-1, "SD: Both SDIO and SPI initialization failed");
            return false;
        }
        util::Debug::info("SD: SPI mode initialized");
    } else {
        util::Debug::info("SD: SDIO mode initialized");
    }
    
    // Print SD card info
    uint32_t cardSize = sd_.card()->sectorCount();
    if (cardSize) {
        float cardSizeGB = 0.000512 * cardSize;
        String infoStr = "SD: Card size: " + String(cardSizeGB) + " GB";
        util::Debug::info(infoStr.c_str());
    }
    
    // Print FAT type
    String fatStr = "SD: Volume is FAT" + String(int(sd_.fatType()));
    util::Debug::info(fatStr.c_str());
    
    // Print free space
    uint32_t freeKB = sd_.vol()->freeClusterCount();
    freeKB *= sd_.vol()->sectorsPerCluster() / 2;
    String spaceStr = "SD: Free space: " + String(freeKB / 1024.0) + " GB";
    util::Debug::info(spaceStr.c_str());
    
    // Run quick write test to verify SD card is working properly using the RingBuf exactly as in the example
    util::Debug::info("SD: Testing RingBuf with SD card...");
    
    // Create a test file
    FsFile testFile;
    if (!testFile.open("sdtest.txt", O_RDWR | O_CREAT | O_TRUNC)) {
        util::Debug::error("SD: Could not create test file");
        return false;
    }
    
    // Initialize a temporary RingBuf
    RingBuf<FsFile, 4096> testBuf;
    testBuf.begin(&testFile);
    
    // Write test data
    uint32_t start = micros();
    for (int i = 0; i < 10; i++) {
        testBuf.print("Test data line ");
        testBuf.println(i);
    }
    
    // Sync the data to the file
    testBuf.sync();
    uint32_t end = micros();
    testFile.close();
    
    float writeTime = (end - start) / 1000000.0;
    String timeStr = "SD: Test write time: " + String(writeTime) + " s";
    util::Debug::info(timeStr.c_str());
    
    // Delete the test file
    sd_.remove("sdtest.txt");
    
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
    
    String infoStr = "SD: Set " + String(channelNames_.size()) + " channel names";
    util::Debug::info(infoStr.c_str());
}

size_t SDWriter::process() {
    // Check for file health
    if (!healthy_) {
        // Try to recover if unhealthy for more than 10 seconds
        if (millis() - lastErrorTime_ > 10000) {
            util::Debug::warning("SD: Attempting recovery after errors");
            closeFile();
            if (!createNewFile()) {
                // Still failing, but don't keep printing errors
                return 0;
            } else {
                // Recovered successfully
                healthy_ = true;
                util::Debug::info("SD: Recovery successful");
            }
        } else {
            // Still in cooldown period, don't retry yet
            return 0;
        }
    }
    
    // Check if file is open and RingBuf is initialized
    if (!dataFile_.isOpen()) {
        if (!createNewFile()) {
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
    
    // Get available samples from the data buffer
    size_t availableSamples = dataBuffer_.available();
    if (availableSamples == 0) {
        // No samples available - check if we need to flush the RingBuf
        if (!dataFile_.isBusy() && ringBuf_->bytesUsed() > 0) {
            syncRingBuf();
        }
        return 0;
    }
    
    // Check if we should start writing (10% threshold or if we were previously full)
    float bufferUtilization = static_cast<float>(availableSamples) / dataBuffer_.capacity();
    bool shouldWrite = bufferUtilization >= config::DATA_BUFFER_WRITE_THRESHOLD || wasBufferFull_;
    
    // Stop writing if buffer nearly empty (hysteresis)
    if (availableSamples * sizeof(data::ChannelSample) < config::MIN_BYTES_FOR_WRITE) {
        shouldWrite = false;
        wasBufferFull_ = false;
    }
    
    if (!shouldWrite) {
        return 0;
    }
    
    // Determine batch size based on available samples
    size_t samplesInBatch = min(availableSamples, static_cast<size_t>(1000)); // Process up to 1000 at a time
    
    // Check if RingBuf has enough space (approx. 50 bytes per sample in CSV)
    size_t ringBufFree = ringBuf_->bytesFree();
    size_t estimatedBatchSize = samplesInBatch * 50; // Estimated CSV size
    
    if (ringBufFree < estimatedBatchSize) {
        // Not enough space in RingBuf - try to write to SD card if not busy
        if (!dataFile_.isBusy() && ringBuf_->bytesUsed() >= config::SD_SECTOR_SIZE) {
            // Write one sector from RingBuf to SD
            if (ringBuf_->writeOut(config::SD_SECTOR_SIZE) != config::SD_SECTOR_SIZE) {
                recordError(-3, "SD: Failed to write sector to SD");
            } else {
                bytesWritten_ += 512;
                lastSuccessfulWrite_ = millis();
            }
        }
        
        // Recalculate available space
        ringBufFree = ringBuf_->bytesFree();
        if (ringBufFree < config::SD_SECTOR_SIZE) {
            // Still not enough space, wait for next cycle
            wasBufferFull_ = true;
            return 0;
        }
        
        // Adjust samplesInBatch based on available RingBuf space
        samplesInBatch = min(samplesInBatch, ringBufFree / 50);
    }
    
    // Read samples from the data buffer and write to RingBuf
    data::ChannelSample sample;
    size_t samplesProcessed = 0;
    
    uint32_t startTime = micros();
    
    for (size_t i = 0; i < samplesInBatch; i++) {
        if (dataBuffer_.read(sample)) {
            if (writeSampleToRingBuf(sample)) {
                samplesProcessed++;
            } else {
                // RingBuf is full, stop processing
                break;
            }
        } else {
            // No more samples available
            break;
        }
    }
    
    uint32_t processingTime = micros() - startTime;
    
    // If we processed samples, update statistics
    if (samplesProcessed > 0) {
        totalSamplesWritten_ += samplesProcessed;
        
        // Only log large batches to reduce noise
        if (samplesProcessed > 100) {
            String procStr = "SD: Processed " + String(samplesProcessed) + 
                             " samples in " + String(processingTime) + " µs";
            util::Debug::detail(procStr.c_str());
        }
    }
    
    // Try to write to SD if not busy and we have at least one sector
    if (!dataFile_.isBusy() && ringBuf_->bytesUsed() >= config::SD_SECTOR_SIZE) {
        // Write one sector from RingBuf to SD
        if (ringBuf_->writeOut(config::SD_SECTOR_SIZE) != config::SD_SECTOR_SIZE) {
            recordError(-4, "SD: Failed to write sector to SD");
        } else {
            bytesWritten_ += 512;
            lastSuccessfulWrite_ = millis();
            totalWrites_++;
        }
    }
    
    // Mark as not full if we processed all available samples
    if (samplesProcessed == availableSamples) {
        wasBufferFull_ = false;
    }
    
    return samplesProcessed;
}

bool SDWriter::createNewFile(bool addHeader) {
    // Close any open file
    closeFile();
    
    // Generate a filename
    currentFilename_ = generateFilename();
    
    String fileStr = "SD: Creating new file: " + String(currentFilename_.c_str());
    util::Debug::info(fileStr.c_str());
    
    // Create the file with appropriate flags
    if (!dataFile_.open(currentFilename_.c_str(), O_RDWR | O_CREAT | O_TRUNC)) {
        recordError(-5, "SD: Failed to create file");
        return false;
    }
    
    // Pre-allocate file space to avoid latency during writes
    String allocStr = "SD: Pre-allocating " + String(config::SD_PREALLOC_SIZE / 1024 / 1024) + " MB";
    util::Debug::detail(allocStr.c_str());
    
    bool preAllocSuccess = dataFile_.preAllocate(config::SD_PREALLOC_SIZE);
    if (!preAllocSuccess) {
        // Log the error but continue - preallocation is optional for functionality
        util::Debug::warning("SD: File preallocation failed - continuing without preallocation");
        // Don't return false here - we'll continue without preallocation
    }
    
    // Initialize RingBuf with the file - THIS MUST HAPPEN BEFORE WRITING!
    ringBuf_->begin(&dataFile_);
    
    // Reset counters
    fileCreationTime_ = millis();
    bytesWritten_ = 0;
    lastWriteTime_ = millis();
    
    // Test the ringbuf with a simple write
    const char* testStr = "# File created with RingBuf\r\n";
    if (ringBuf_->write(testStr, strlen(testStr)) != strlen(testStr)) {
        recordError(-9, "SD: Failed initial RingBuf write test");
        closeFile();
        return false;
    }
    
    // Write header if requested
    if (addHeader) {
        if (!writeHeader()) {
            recordError(-7, "SD: Failed to write header");
            closeFile();
            return false;
        }
        
        // Force a sync to make sure header is written
        if (!dataFile_.isBusy()) {
            size_t bytesToWrite = ringBuf_->bytesUsed();
            if (bytesToWrite > 0) {
                size_t written = ringBuf_->writeOut(bytesToWrite);
                util::Debug::detail("SD: Initial header sync wrote " + String(written) + " bytes");
                bytesWritten_ += written;
            }
        }
    }
    
    // Update status
    lastSuccessfulWrite_ = millis();
    consecutiveErrors_ = 0;
    
    // Return success even if preallocation failed
    return true;
}

bool SDWriter::closeFile() {
    // Sync any remaining data in the RingBuf
    if (dataFile_.isOpen()) {
        // First try to flush only the complete sectors
        syncRingBuf();
        
        // Then do a full sync to get any remaining partial data
        if (ringBuf_->bytesUsed() > 0) {
            ringBuf_->sync();
        }
        
        // Truncate file to actual data size and close
        dataFile_.truncate();
        dataFile_.close();
        
        String closeStr = "SD: Closed file: " + String(currentFilename_.c_str()) + 
                        ", bytes written: " + String(bytesWritten_);
        util::Debug::info(closeStr.c_str());
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
    if (millis() - fileCreationTime_ >= config::SD_FILE_ROTATION_INTERVAL_MS) {
        return true;
    }
    
    // Also rotate file if it gets too large (90% of preallocation)
    if (bytesWritten_ >= config::SD_PREALLOC_SIZE * 0.9) {
        return true;
    }
    
    return false;
}

bool SDWriter::flush() {
    return syncRingBuf();
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
    char filename[config::SD_MAX_FILENAME_LENGTH];
    
    // Get current time
    time_t t = now();
    
    // Format as YYYYMMDD_HHMMSS.csv
    snprintf(filename, sizeof(filename), "data_%04d%02d%02d_%02d%02d%02d.csv",
            year(t), month(t), day(t),
            hour(t), minute(t), second(t));
    
    return std::string(filename);
}

bool SDWriter::writeHeader() {
    // Create a simple, clear CSV header
    std::string header = "timestamp_us,channel_index,raw_value";
    
    // Add channel name header if we have channel names
    if (!channelNames_.empty()) {
        header += ",channel_name";
    }
    
    // Add newline (CRLF for compatibility with various tools)
    header += "\r\n";
    
    // Write to RingBuf - use direct write to ensure proper handling
    size_t headerLen = header.length();
    size_t bytesWritten = ringBuf_->write(header.c_str(), headerLen);
    
    if (bytesWritten != headerLen) {
        util::Debug::error("SD: Failed to write header, only wrote " + 
                          String(bytesWritten) + " of " + String(headerLen) + " bytes");
        return false;
    }
    
    return true;
}

bool SDWriter::writeSampleToRingBuf(const data::ChannelSample& sample) {
    // Format the sample as CSV
    std::string line = formatSampleAsCsv(sample);
    
    // Write to RingBuf
    size_t bytesWritten = ringBuf_->write(line.c_str(), line.length());
    
    // Check for write errors
    if (bytesWritten != line.length()) {
        if (ringBuf_->getWriteError()) {
            util::Debug::warning("SD: RingBuf write error");
            ringBuf_->clearWriteError();
        }
        return false;
    }
    
    return true;
}

std::string SDWriter::formatSampleAsCsv(const data::ChannelSample& sample) {
    // Format basic sample data with fixed field width for better alignment
    char buffer[128];
    
    // Use fixed precision formatting for timestamp, channel, and value
    // Make sure all values are properly converted to strings
    snprintf(buffer, sizeof(buffer), "%llu,%u,%lu", 
             sample.timestamp, sample.channelIndex, sample.rawValue);
    std::string line = buffer;
    
    // Add channel name if available
    if (sample.channelIndex < channelNames_.size()) {
        line += ",";
        line += channelNames_[sample.channelIndex];
    }
    
    // Add CRLF for compatibility with more tools
    line += "\r\n";
    
    return line;
}

bool SDWriter::syncRingBuf() {
    if (!dataFile_.isOpen()) {
        return false;
    }
    
    size_t bytesUsed = ringBuf_->bytesUsed();
    if (bytesUsed == 0) {
        return true;
    }
    
    uint32_t startTime = micros();
    
    // Follow the same approach as the example - just call sync()
    bool success = ringBuf_->sync();
    
    uint32_t syncTime = micros() - startTime;
    
    if (success) {
        bytesWritten_ += bytesUsed;
        lastSuccessfulWrite_ = millis();
        lastWriteTime_ = millis();
        
        if (syncTime > maxWriteTime_) {
            maxWriteTime_ = syncTime;
            String timeStr = "SD: New max sync time: " + String(maxWriteTime_) + " µs";
            util::Debug::detail(timeStr.c_str());
        }
        
        // Only log larger syncs
        if (bytesUsed > 10000) {
            String syncStr = "SD: Synced " + String(bytesUsed) + 
                             " bytes in " + String(syncTime) + " µs";
            util::Debug::detail(syncStr.c_str());
        }
        
        return true;
    } else {
        recordError(-8, "SD: Failed to sync RingBuf");
        return false;
    }
}

void SDWriter::recordError(int errorCode, const char* errorMessage) {
    lastError_ = errorCode;
    lastErrorTime_ = millis();
    consecutiveErrors_++;
    totalErrors_++;
    
    // Only log every few errors to prevent log spam
    if (consecutiveErrors_ == 1 || consecutiveErrors_ % 10 == 0) {
        // Convert Arduino String to std::string for logging
        String errorMsg = String("SD ERROR #") + String(totalErrors_) + 
                          ": " + String(errorMessage) + 
                          " (code " + String(errorCode) + ")";
        util::Debug::error(errorMsg.c_str());
    }
    
    // Mark as unhealthy if too many consecutive errors
    if (consecutiveErrors_ >= 3) {
        healthy_ = false;
    }
}

} // namespace storage
} // namespace baja