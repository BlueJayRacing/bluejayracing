#include "storage/sd_writer.hpp"
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
      wasBufferFull_(false),
      isFirstFile_(true) {
}

SDWriter::~SDWriter() {
    closeFile();
}

bool SDWriter::begin(uint8_t chipSelect) {
    util::Debug::info(F("SD: Initializing"));
    
    // Try SDIO mode first, then fall back to SPI if needed
    if (!sd_.begin(SdioConfig(FIFO_SDIO))) {
        if (!sd_.begin(SdSpiConfig(chipSelect, SHARED_SPI, SD_SCK_MHZ(50)))) {
            util::Debug::error(F("SD: Both SDIO and SPI initialization failed"));
            return false;
        }
        util::Debug::info(F("SD: SPI mode initialized"));
    } else {
        util::Debug::info(F("SD: SDIO mode initialized"));
    }
    
    // Print SD card info
    uint32_t cardSize = sd_.card()->sectorCount();
    if (cardSize) {
        float cardSizeGB = 0.000512 * cardSize;
        util::Debug::info(F("SD: Card size: ") + String(cardSizeGB) + F(" GB"));
    }
    
    // Print FAT type
    util::Debug::info(F("SD: Volume is FAT") + String(int(sd_.fatType())));
    
    // Print free space
    uint32_t freeKB = sd_.vol()->freeClusterCount();
    freeKB *= sd_.vol()->sectorsPerCluster() / 2;
    util::Debug::info(F("SD: Free space: ") + String(freeKB / 1024.0) + F(" GB"));
    
    // Run quick write test to verify SD card is working properly
    util::Debug::info(F("SD: Testing write capability..."));
    
    // Create a test file
    FsFile testFile;
    if (!testFile.open("sdtest.txt", O_RDWR | O_CREAT | O_TRUNC)) {
        util::Debug::error(F("SD: Could not create test file"));
        return false;
    }
    
    // Initialize the RingBuf with the file
    RingBuf<FsFile, 4096> testBuf;
    testBuf.begin(&testFile);
    
    // Write test data
    uint32_t start = micros();
    for (int i = 0; i < 10; i++) {
        testBuf.print(F("Test data line "));
        testBuf.println(i);
    }
    
    // Sync the data to the file
    testBuf.sync();
    uint32_t end = micros();
    testFile.close();
    
    float writeTime = (end - start) / 1000000.0;
    util::Debug::info(F("SD: Test write time: ") + String(writeTime) + " s");
    
    // Delete the test file
    sd_.remove("sdtest.txt");
    
    healthy_ = true;
    lastSuccessfulWrite_ = millis();
    isFirstFile_ = true;
    
    return true;
}

void SDWriter::setChannelNames(const std::vector<adc::ChannelConfig>& channelConfigs) {
    channelNames_.clear();
    for (const auto& config : channelConfigs) {
        if (config.enabled) {
            channelNames_.push_back(config.name);
        }
    }
    
    util::Debug::info(F("SD: Set ") + String(channelNames_.size()) + F(" channel names"));
}

size_t SDWriter::process() {
    // Handle health recovery if needed
    if (!healthy_) {
        if (millis() - lastErrorTime_ > 10000) {
            util::Debug::warning("SD: Attempting recovery after errors");
            closeFile();
            if (!createNewFile()) {
                return 0;
            } else {
                healthy_ = true;
                util::Debug::info("SD: Recovery successful");
            }
        } else {
            return 0;
        }
    }
    
    // Check if file is open
    if (!dataFile_.isOpen()) {
        if (!createNewFile()) {
            return 0;
        }
        return 0;
    }
    
    // Check if we need to rotate the file
    if (shouldRotateFile()) {
        closeFile();
        if (!createNewFile()) {
            return 0;
        }
        return 0;
    }
    
    // Get available samples from the data buffer
    size_t availableSamples = dataBuffer_.available();
    if (availableSamples == 0) {
        // No samples available - check if we need to flush the RingBuf
        if (ringBuf_->bytesUsed() > 0) {
            // For the first file, or if we haven't written in a while, do a full sync
            if (isFirstFile_ || (millis() - lastWriteTime_ > 5000)) {
                syncRingBuf(true); // Force full sync
            } else if (!dataFile_.isBusy()) {
                baja::util::Debug::info("SD: Syncing RingBuf, no samples");
                syncRingBuf(false); // Normal sync of complete sectors
            } else { // ring buffer is busy
                baja::util::Debug::info("no sample available, ring buffer is busy");
            }
        }
        return 0;
    }
    
    // Check if we should start writing (based on threshold)
    float bufferUtilization = static_cast<float>(availableSamples) / dataBuffer_.capacity();
    bool shouldWrite = bufferUtilization >= config::DATA_BUFFER_WRITE_THRESHOLD || wasBufferFull_;
    
    // For first file, be more aggressive with writing to ensure data gets stored
    if (isFirstFile_ && availableSamples > 0) {
        shouldWrite = true;
    }
    
    // Stop writing if buffer nearly empty (hysteresis)
    if (availableSamples * sizeof(data::ChannelSample) < config::MIN_BYTES_FOR_WRITE && !isFirstFile_) {
        shouldWrite = false;
        wasBufferFull_ = false;
    }
    
    if (!shouldWrite) {
        return 0;
    }
    
    // Determine batch size based on available samples
    size_t samplesInBatch = min(availableSamples, static_cast<size_t>(15)); ///////////////////////////////////////////////////////////// set samples smaler
    
    // For first file, process smaller batches to ensure quicker writes
    if (isFirstFile_) {
        samplesInBatch = min(samplesInBatch, static_cast<size_t>(25));
    }
    
    // Ensure RingBuf has enough space (approx. 50 bytes per sample in CSV)
    size_t ringBufFree = ringBuf_->bytesFree();
    size_t estimatedBatchSize = samplesInBatch * 50;
    
    if (ringBufFree < estimatedBatchSize) {
        // Not enough space - force a sync
        syncRingBuf(true);
        
        // Recalculate available space
        ringBufFree = ringBuf_->bytesFree();
        if (ringBufFree < 1024) { // Minimum required space
            wasBufferFull_ = true;
            return 0;
        }
        
        // Adjust batch size based on available space
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
                break;
            }
        } else {
            break;
        }
    }
    
    uint32_t processingTime = micros() - startTime;
    
    // Update statistics if samples were processed
    if (samplesProcessed > 0) {
        totalSamplesWritten_ += samplesProcessed;
        
        if (samplesProcessed > 50) {
            util::Debug::detail("SD: Processed " + String(samplesProcessed) + 
                             " samples in " + String(processingTime) + " µs");
        }
    }
    
    // For first file, sync after every batch to ensure data is written
    if (isFirstFile_ && samplesProcessed > 0) {
        syncRingBuf(true);
        
        // After a successful first write with sync, it's no longer the "first file"
        if (bytesWritten_ > 1024) {
            isFirstFile_ = false;
            util::Debug::info("SD: First file successfully initialized");
        }
    } 
    // For normal operation, write to SD if we have a complete sector
    else if (!dataFile_.isBusy() && ringBuf_->bytesUsed() >= config::SD_SECTOR_SIZE) {
        syncRingBuf(false);
    }
    
    // Update buffer state
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
    
    util::Debug::info("SD: Creating new file: " + String(currentFilename_.c_str()));
    
    // Create the file
    if (!dataFile_.open(currentFilename_.c_str(), O_RDWR | O_CREAT | O_TRUNC)) {
        recordError(-5, "SD: Failed to create file");
        return false;
    }
    
    // CRITICAL: Initialize RingBuf with the file BEFORE preallocation
    ringBuf_->begin(&dataFile_);
    
    // Pre-allocate file space to reduce fragmentation
    util::Debug::detail("SD: Pre-allocating " + String(config::SD_PREALLOC_SIZE / 1024 / 1024) + " MB");
    
    bool preAllocSuccess = dataFile_.preAllocate(config::SD_PREALLOC_SIZE);
    if (!preAllocSuccess) {
        util::Debug::warning("SD: File preallocation failed - continuing without preallocation");
    } else {
        util::Debug::info("SD: Preallocation successful");
    }
    
    // Reset counters
    fileCreationTime_ = millis();
    bytesWritten_ = 0;
    lastWriteTime_ = millis();
    
    // Write header if requested (through the RingBuf)
    if (addHeader) {
        if (!writeHeader()) {
            recordError(-7, "SD: Failed to write header");
            closeFile();
            return false;
        }
        
        // Force a sync to make sure header is written
        if (!ringBuf_->sync()) {
            recordError(-9, "SD: Failed to sync header");
            closeFile();
            return false;
        } else {
            // Update bytes written
            bytesWritten_ = dataFile_.position();
        }
    }
    
    // Mark as first file for special handling
    // isFirstFile_ = true;
    
    // Verify the file is open and valid
    if (!dataFile_.isOpen()) {
        recordError(-10, "SD: File not open after creation");
        return false;
    }
    
    // Update status
    lastSuccessfulWrite_ = millis();
    consecutiveErrors_ = 0;
    
    return true;
}

bool SDWriter::closeFile() {
    // Sync any remaining data in the RingBuf
    if (dataFile_.isOpen()) {
        // Ensure all data is synced
        if (ringBuf_->bytesUsed() > 0) {
            ringBuf_->sync();
        }
        
        // Flush any remaining file buffers
        dataFile_.flush();
        
        // Get final position for byte count
        bytesWritten_ = dataFile_.position();
        
        // Truncate file to actual data size and close
        dataFile_.truncate();
        dataFile_.close();
        
        util::Debug::info("SD: Closed file: " + String(currentFilename_.c_str()) + 
                      ", bytes written: " + String(bytesWritten_));
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
    // Check file age
    if (millis() - fileCreationTime_ >= config::SD_FILE_ROTATION_INTERVAL_MS) {
        return true;
    }
    
    // Check file size
    if (bytesWritten_ >= config::SD_PREALLOC_SIZE * 0.9) {
        return true;
    }
    
    return false;
}

bool SDWriter::flush() {
    return syncRingBuf(true);
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
    // Create CSV header
    std::string header = "timestamp_us,channel_index,raw_value";
    
    // Add channel name header if available
    if (!channelNames_.empty()) {
        header += ",channel_name";
    }
    
    // Add newline (CRLF for compatibility)
    header += "\r\n";
    
    // Write to RingBuf
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
    // Format CSV line efficiently using a buffer
    char buffer[128];
    
    // Format timestamp, channel, and value
    int len = snprintf(buffer, sizeof(buffer), "%llu,%u,%lu", 
             sample.timestamp, sample.channelIndex, sample.rawValue);
    
    // Add channel name if available
    if (sample.channelIndex < channelNames_.size()) {
        len += snprintf(buffer + len, sizeof(buffer) - len, ",%s", 
                      channelNames_[sample.channelIndex].c_str());
    }
    
    // Add CRLF
    len += snprintf(buffer + len, sizeof(buffer) - len, "\r\n");
    
    // Write to RingBuf
    size_t bytesWritten = ringBuf_->write(buffer, len);
    
    // Check for errors
    if (bytesWritten != len) {
        if (ringBuf_->getWriteError()) {
            util::Debug::warning("SD: RingBuf write error");
            ringBuf_->clearWriteError();
        }
        return false;
    }
    
    return true;
}

bool SDWriter::syncRingBuf(bool forceFullSync) {
    if (!dataFile_.isOpen()) {
        return false;
    }
    
    size_t bytesUsed = ringBuf_->bytesUsed();
    if (bytesUsed == 0) {
        return true;
    }
    
    uint32_t startTime = micros();
    bool success = false;
    size_t bytesWritten = 0;
    bool syncType = false;
    
    // Force a full sync if requested, otherwise only write complete sectors
    if (forceFullSync) {
        syncType = true;
        // Use full sync for guaranteed writes (slightly slower)
        success = ringBuf_->sync();
        bytesWritten = bytesUsed; // All bytes should be written
    } else {
        // Only write complete sectors for efficiency
        size_t alignedBytes = (bytesUsed / config::SD_SECTOR_SIZE) * config::SD_SECTOR_SIZE;
        
        // Write aligned bytes if we have at least one sector
        if (alignedBytes >= config::SD_SECTOR_SIZE) {
            bytesWritten = ringBuf_->writeOut(alignedBytes);
            success = (bytesWritten == alignedBytes);
        } else {
            // Not enough for a full sector, but return true anyway
            return true;
        }
    }
    
    uint32_t syncTime = micros() - startTime;
    
    if (success) {
        // Update bytes written based on file position - more accurate
        bytesWritten_ = dataFile_.position();
        lastSuccessfulWrite_ = millis();
        lastWriteTime_ = millis();
        
        if (syncTime > maxWriteTime_) {
            maxWriteTime_ = syncTime;
            util::Debug::info("SD: Max write time: " + String(maxWriteTime_) + " µs");
        }
        String a = (syncType? "Force":"write");
        // Only print the log if either bytesWritten > 500 or syncTime > 100 AND the calculated speed is below 20 MB/s
        if ((bytesWritten > 1000 || syncTime > 30) && (static_cast<float>(bytesWritten) / syncTime < 20.0) || syncTime > 50) {
            util::Debug::info("SD: " + a + "-Synced " + String(bytesWritten) + 
                            " bytes in " + String(syncTime) + " µs");
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
    
    // Limit error logging to prevent spam
    if (consecutiveErrors_ == 1 || consecutiveErrors_ % 10 == 0) {
        String errorMsg = String("SD ERROR #") + String(totalErrors_) + 
                        ": " + String(errorMessage) + 
                        " (code " + String(errorCode) + ")";
        util::Debug::error(errorMsg.c_str());
    }
    
    // Mark as unhealthy after multiple errors
    if (consecutiveErrors_ >= 3) {
        healthy_ = false;
    }
}

} // namespace storage
} // namespace baja