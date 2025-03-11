#include "sd_writer.hpp"
#include <TimeLib.h>

namespace baja {
namespace storage {

SDWriter::SDWriter(buffer::RingBuffer<data::ChannelSample, adc::RING_BUFFER_SIZE>& ringBuffer)
    : ringBuffer_(ringBuffer),
      writeBufferPos_(0),
      fileCreationTime_(0),
      bytesWritten_(0) {
    // We'll use the global buffers when needed, no need to copy them here
}

SDWriter::~SDWriter() {
    // Close any open file
    closeFile();
}

bool SDWriter::begin(uint8_t chipSelect) {
    // Initialize SD card in SDIO mode
    if (!sd_.begin(SdioConfig(FIFO_SDIO))) {
        // If SDIO fails, try SPI mode as fallback
        if (!sd_.begin(SdSpiConfig(chipSelect, SHARED_SPI, SD_SCK_MHZ(50)))) {
            return false;
        }
    }
    
    // Configure SD card for optimal performance
    // Note: Teensy 4.1 SDIO is already configured with appropriate priority by the SdFat library
    
    return true;
}

void SDWriter::setChannelNames(const std::vector<adc::ChannelConfig>& channelConfigs) {
    channelNames_.clear();
    for (const auto& config : channelConfigs) {
        if (config.enabled) {
            channelNames_.push_back(config.name);
        }
    }
}

size_t SDWriter::process() {
    // Check if we need to rotate the file
    if (shouldRotateFile()) {
        closeFile();
        createNewFile();
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
        return 0;
    }
    
    // Limit to how many we can read at once
    size_t samplesToRead = min(availableSamples, adc::SAMPLES_PER_SD_BLOCK);
    size_t samplesRead = ringBuffer_.readMultiple(sdSampleBuffer, samplesToRead);
    
    // Process each sample
    size_t samplesWritten = 0;
    for (size_t i = 0; i < samplesRead; i++) {
        // Format as CSV
        std::string line = sdSampleBuffer[i].toCSV() + "\n";
        
        // Add to the write buffer
        if (addLineToBuffer(line)) {
            samplesWritten++;
        }
    }
    
    // Check if buffer is near full and should be flushed
    if (writeBufferPos_ >= adc::SD_BLOCK_SIZE * 0.9) {
        flushBuffer();
    }
    
    return samplesWritten;
}

bool SDWriter::createNewFile(bool addHeader) {
    // Close any open file
    closeFile();
    
    // Generate a filename
    currentFilename_ = generateFilename();
    
    // Create the file
    if (!dataFile_.open(currentFilename_.c_str(), O_WRONLY | O_CREAT | O_TRUNC)) {
        return false;
    }
    
    // Reset counters
    fileCreationTime_ = millis();
    bytesWritten_ = 0;
    writeBufferPos_ = 0;
    
    // Write header if requested
    if (addHeader && !writeHeader()) {
        closeFile();
        return false;
    }
    
    return true;
}

bool SDWriter::closeFile() {
    // Flush any remaining data
    flushBuffer();
    
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
    
    return false;
}

bool SDWriter::flush() {
    return flushBuffer();
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
    
    // Disable interrupts during SD write to prevent corruption
    noInterrupts();
    
    // Write the buffer to the file
    size_t bytesWritten = dataFile_.write(sdWriterBuffer, writeBufferPos_);
    
    // Re-enable interrupts
    interrupts();
    
    // Update counters
    bytesWritten_ += bytesWritten;
    writeBufferPos_ = 0;
    
    // Sync file occasionally to prevent data loss on power failure
    if (bytesWritten_ % (adc::SD_BLOCK_SIZE * 10) == 0) {
        dataFile_.sync();
    }
    
    return (bytesWritten == writeBufferPos_);
}

bool SDWriter::addLineToBuffer(const std::string& line) {
    // Check if line will fit in the buffer
    if (writeBufferPos_ + line.length() > adc::SD_BLOCK_SIZE) {
        // Buffer is full, flush it first
        if (!flushBuffer()) {
            return false;
        }
    }
    
    // Add line to buffer
    memcpy((char*)sdWriterBuffer + writeBufferPos_, line.c_str(), line.length());
    writeBufferPos_ += line.length();
    
    return true;
}

} // namespace storage
} // namespace baja