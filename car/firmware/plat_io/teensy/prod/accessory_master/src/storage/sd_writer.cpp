#include "storage/sd_writer.hpp"
#include <TimeLib.h>

namespace baja {
namespace storage {

Threads::Mutex SDWriter::mutex_;

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
    isFirstFile_(true),
    lastPeriodicSyncTime_(0),
    totalSyncTime_(0),
    syncCount_(0),
    maxSyncTime_(0),
    performingFileOperation_(false),
    needDataSync_(false) {
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
    // Clear any existing names
    channelNames_.clear();
    
    // Resize the vector to handle all possible ADC channels (0-15)
    channelNames_.resize(16);
    
    // Initialize all to empty strings
    for (size_t i = 0; i < channelNames_.size(); i++) {
        channelNames_[i] = "";
    }
    
    // Add channel names at their respective indices
    size_t enabledCount = 0;
    for (const auto& config : channelConfigs) {
        // Store the channel name at its actual ADC index
        if (config.channelIndex < channelNames_.size()) {
            channelNames_[config.channelIndex] = config.name;
            if (config.enabled) {
                enabledCount++;
            }
        }
    }
    
    util::Debug::info(F("SD: Set ") + String(enabledCount) + F(" enabled channel names out of ") + 
                   String(channelNames_.size()) + F(" total channels"));
}

size_t SDWriter::process() {
    static int deferCount = 0;
    // Skip processing if we're in the middle of a file operation
    if (performingFileOperation_) {
        
        
        if (mutex_.getState() == 0) {
            deferCount--;
            if (deferCount == 0) {
                Serial.println("Sync In Progress, skipping, and unlocking");
                performingFileOperation_ = false;
            }
        }
        return 0;
    }

    // Health recovery and file handling
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


    // Periodic sync check - perform regular syncs to ensure data is written
    uint32_t currentTime = millis();
    if (currentTime - lastPeriodicSyncTime_ >= config::SD_SYNC_INTERVAL_MS) {
        Serial.println("Syncing directory !!!!!!!!!!!!!!!!!!!!!");
        // Only perform sync if not busy 
        if (!dataFile_.isBusy()) {
            util::Debug::detail("SD: Performing periodic sync");
            
            // Benchmark the sync operation
            uint32_t syncStartTime = micros();
            startAsyncSync(dataFile_);
            deferCount = 3000;
            uint32_t syncDuration = micros() - syncStartTime;
            
            // Update sync statistics
            lastPeriodicSyncTime_ = currentTime;
            totalSyncTime_ += syncDuration;
            syncCount_++;
            
            if (syncDuration > maxSyncTime_) {
                maxSyncTime_ = syncDuration;
            }
            
            // Log warning if sync took too long
            if (syncDuration > config::SD_MAX_SYNC_TIME_US) {
                util::Debug::warning("SD: Long sync time: " + String(syncDuration) + " us");
                Serial.print("!");
            } else {
                util::Debug::detail("SD: Periodic sync completed in " + String(syncDuration) + " ms");
                Serial.print("ok");
            }
            
        } else {
            // Mark that we need a sync when card is not busy
            needDataSync_ = true;
        }
        return 0;
    }

    // Process deferred sync if needed and card is not busy
    if (needDataSync_ && !dataFile_.isBusy()) {
        Serial.println("Syncing directory !!!!!!!!!!!!!!!!!!!!!");
        uint32_t syncStartTime = micros();
        startAsyncSync(dataFile_);
        deferCount = 3000;
        Serial.println("Took " + String(micros() - syncStartTime) + " us to sync");
        needDataSync_ = false;
        lastPeriodicSyncTime_ = currentTime;
        return 0;
    }


    // Get available samples from the data buffer
    size_t availableSamples = dataBuffer_.available();
    if (availableSamples == 0) {
        if (ringBuf_->bytesUsed() > 0) {
            if (isFirstFile_ || (millis() - lastWriteTime_ > 5000)) {
                syncRingBuf(true); // Force full sync
            } else if (!dataFile_.isBusy()) {
                baja::util::Debug::info("SD: Syncing RingBuf, no samples");
                syncRingBuf(false); // Normal sync of complete sectors
            } else {
                baja::util::Debug::info("no sample available, ring buffer is busy");
            }
        }
        return 0;
    }

    // Decide whether to write based on threshold
    float bufferUtilization = static_cast<float>(availableSamples) / dataBuffer_.capacity();
    bool shouldWrite = (bufferUtilization >= config::DATA_BUFFER_WRITE_THRESHOLD) || wasBufferFull_;
    if (isFirstFile_ && availableSamples > 0) {
        shouldWrite = true;
    }
    if (availableSamples * sizeof(data::ChannelSample) < config::MIN_BYTES_FOR_WRITE && !isFirstFile_) {
        shouldWrite = false;
        wasBufferFull_ = false;
    }
    if (!shouldWrite) {
        return 0;
    }

    // Determine batch size based on available samples
    size_t samplesInBatch = min(availableSamples, static_cast<size_t>(15));
    if (isFirstFile_) {
        samplesInBatch = min(samplesInBatch, static_cast<size_t>(25));
    }

    // Ensure RingBuf has enough space (approx. 50 bytes per sample in CSV)
    size_t ringBufFree = ringBuf_->bytesFree();
    size_t estimatedBatchSize = samplesInBatch * 50;
    if (ringBufFree < estimatedBatchSize) {
        // Not enough space - force a sync
        syncRingBuf(true);
        ringBufFree = ringBuf_->bytesFree();
        if (ringBufFree < 1024) { // Minimum required space
            wasBufferFull_ = true;
            return 0;
        }
        samplesInBatch = min(samplesInBatch, ringBufFree / 50);
    }

    // Process a batch of samples
    size_t samplesProcessed = 0;
    for (size_t i = 0; i < samplesInBatch; i++) {
        data::ChannelSample sample;
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

    // Update statistics if samples were processed
    if (samplesProcessed > 0) {
        totalSamplesWritten_ += samplesProcessed;
        if (samplesProcessed > 50) {
            util::Debug::detail("SD: Processed " + String(samplesProcessed) + " samples");
        }
    }

    // Sync RingBuf based on operation mode and file status
    if (isFirstFile_ && samplesProcessed > 0) {
        syncRingBuf(true);
        if (bytesWritten_ > 1024) {
            isFirstFile_ = false;
            util::Debug::info("SD: First file successfully initialized");
        }
    } else if (!dataFile_.isBusy() && ringBuf_->bytesUsed() >= config::SD_SECTOR_SIZE) {
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
    
    uint32_t preAllocStart = micros();
    bool preAllocSuccess = dataFile_.preAllocate(config::SD_PREALLOC_SIZE);
    Serial.println("Pre-alloc time: " + String(micros() - preAllocStart) + " us");

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
    if (config::CUSTOM_STRING_CONVERSION_ROUTINE) {
        char buffer[128];
        size_t pos = 0;
        size_t remaining = sizeof(buffer);

        // A helper lambda to convert an unsigned integer (up to 64 bits) to a decimal string.
        // It writes into 'dest' (which has size destSize) and returns the number of characters written.
        auto u64_to_str = [](uint64_t value, char* dest, size_t destSize) -> size_t {
            char temp[21]; // Maximum 20 digits for 64-bit number.
            size_t tempPos = 0;
            if (value == 0) {
                if (destSize > 0) {
                    dest[0] = '0';
                    return 1;
                }
                return 0;
            }
            // Write digits in reverse order.
            while (value > 0 && tempPos < sizeof(temp)) {
                temp[tempPos++] = '0' + (value % 10);
                value /= 10;
            }
            // Now copy them in correct order.
            size_t len = tempPos;
            if (len > destSize) len = destSize;
            for (size_t i = 0; i < len; i++) {
                dest[i] = temp[len - 1 - i];
            }
            return len;
        };

        // Convert and write the timestamp.
        size_t n = u64_to_str(sample.timestamp, buffer + pos, remaining);
        pos += n;
        remaining = sizeof(buffer) - pos;
        if (remaining > 0) {
            buffer[pos++] = ',';
            remaining--;
        }

        // Convert and write the channel index.
        n = u64_to_str(sample.channelIndex, buffer + pos, remaining);
        pos += n;
        remaining = sizeof(buffer) - pos;
        if (remaining > 0) {
            buffer[pos++] = ',';
            remaining--;
        }

        // Convert and write the raw ADC value.
        n = u64_to_str(sample.rawValue, buffer + pos, remaining);
        pos += n;
        remaining = sizeof(buffer) - pos;

        // Add channel name if available and not empty
        // FIXED: Ensure we check the channel index is in range AND the name isn't empty
        if (sample.channelIndex < channelNames_.size() && !channelNames_[sample.channelIndex].empty()) {
            if (remaining > 0) {
                buffer[pos++] = ',';
                remaining--;
            }
            
            const std::string& chName = channelNames_[sample.channelIndex];
            size_t nameLen = chName.length();
            if (nameLen > remaining)
                nameLen = remaining;
            
            memcpy(buffer + pos, chName.c_str(), nameLen);
            pos += nameLen;
            remaining = sizeof(buffer) - pos;
        }

        // Append CRLF.
        const char* crlf = "\r\n";
        size_t crlfLen = 2;
        if (crlfLen > remaining)
            crlfLen = remaining;
        memcpy(buffer + pos, crlf, crlfLen);
        pos += crlfLen;

        // Write the complete CSV line to the ring buffer.
        size_t bytesWritten = ringBuf_->write(buffer, pos);
        if (bytesWritten != pos) {
            if (ringBuf_->getWriteError()) {
                util::Debug::warning("SD: RingBuf write error");
                ringBuf_->clearWriteError();
            }
            return false;
        }
        return true;
    } else {
        // Format CSV line efficiently using a buffer
        char buffer[128];
        
        // Format timestamp, channel, and value
        int len = snprintf(buffer, sizeof(buffer), "%llu,%u,%lu", 
                sample.timestamp, sample.channelIndex, sample.rawValue);
        
        // Add channel name if available
        // FIXED: Ensure we check the channel index is in range AND the name isn't empty
        if (sample.channelIndex < channelNames_.size() && !channelNames_[sample.channelIndex].empty()) {
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
}

bool SDWriter::syncRingBuf(bool forceFullSync) {
    if (!dataFile_.isOpen()) {
        return false;
    }
    
    size_t bytesUsed = ringBuf_->bytesUsed();
    if (bytesUsed == 0) {
        return true;
    }
    
    if (!forceFullSync && dataFile_.isBusy()) {
        return false; // Card is busy, try again later
    }
    
    uint32_t startTime = micros();
    bool success = false;
    size_t bytesWritten = 0;
    bool syncType = false;
    
    // Force a full sync if requested, otherwise only write one sector
    if (forceFullSync) {
        syncType = true;
        // Use full sync for guaranteed writes (slightly slower)
        success = ringBuf_->sync();
        bytesWritten = bytesUsed; // All bytes should be written
    } else {
        // Only write exactly ONE sector for efficiency
        if (bytesUsed >= config::SD_SECTOR_SIZE) {
            // Only write exactly ONE sector at a time
            bytesWritten = ringBuf_->writeOut(config::SD_SECTOR_SIZE);
            success = (bytesWritten == config::SD_SECTOR_SIZE);
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

void SDWriter::asyncSyncTask(void* arg) {
    threads.setSliceMicros(50);
    // Cast the argument to a pointer to FsFile.
    FsFile* file = static_cast<FsFile*>(arg);
    // Call the blocking sync() operation.
    bool syncResult = file->sync();
    // Optionally, print the result for debugging.
    Serial.print("Async sync result: ");
    Serial.println(syncResult ? "Success" : "Failure");
    // When the function returns, the thread exits.
    mutex_.unlock();
}
  
// This function starts the async sync operation.
void SDWriter::startAsyncSync(FsFile &file) {
    performingFileOperation_ = true;
    mutex_.lock();
    // Use the global TeensyThreads instance to add a new thread.
    threads.addThread(asyncSyncTask, &file);
}

} // namespace storage
} // namespace baja