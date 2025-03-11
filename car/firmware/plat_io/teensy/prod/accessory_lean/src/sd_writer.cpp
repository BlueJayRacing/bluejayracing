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
    Serial.println("  SD: Initializing with chipSelect=" + String(chipSelect));
    
    // Initialize SD card in SDIO mode
    if (!sd_.begin(SdioConfig(FIFO_SDIO))) {
        Serial.println("  SD: SDIO mode failed, trying SPI mode");
        // If SDIO fails, try SPI mode as fallback
        if (!sd_.begin(SdSpiConfig(chipSelect, SHARED_SPI, SD_SCK_MHZ(50)))) {
            Serial.println("  SD: SPI mode failed too");
            return false;
        }
        Serial.println("  SD: SPI mode succeeded");
    } else {
        Serial.println("  SD: SDIO mode succeeded");
    }
    
    // Print SD card info for debugging
    Serial.println("  SD: Card successfully mounted");
    
    cid_t cid;
    if (sd_.card()->readCID(&cid)) {
        Serial.print("  SD: Manufacturer ID: ");
        Serial.println(int(cid.mid), HEX);
        Serial.print("  SD: Product name: ");
        for (uint8_t i = 0; i < 5; i++) {
            Serial.print(char(cid.pnm[i]));
        }
        Serial.println();
    }
    
    uint32_t cardSize = sd_.card()->sectorCount();
    if (cardSize) {
        float cardSizeGB = 0.000512 * cardSize;
        Serial.print("  SD: Card size: ");
        Serial.print(cardSizeGB);
        Serial.println(" GB");
    }
    
    Serial.print("  SD: Volume is FAT");
    Serial.println(int(sd_.fatType()));
    
    uint32_t freeKB = sd_.vol()->freeClusterCount();
    freeKB *= sd_.vol()->sectorsPerCluster() / 2;
    Serial.print("  SD: Free space: ");
    Serial.print(freeKB / 1024.0);
    Serial.println(" GB");
    
    return true;
}

void SDWriter::setChannelNames(const std::vector<adc::ChannelConfig>& channelConfigs) {
    channelNames_.clear();
    for (const auto& config : channelConfigs) {
        if (config.enabled) {
            channelNames_.push_back(config.name);
        }
    }
    
    Serial.print("  SD: Set ");
    Serial.print(channelNames_.size());
    Serial.println(" channel names");
}

size_t SDWriter::process() {
    // Check if we need to rotate the file
    if (shouldRotateFile()) {
        Serial.println("  SD: Rotating file");
        closeFile();
        createNewFile();
    }
    
    // Check if file is open
    if (!dataFile_.isOpen()) {
        Serial.println("  SD: File not open, creating new file");
        if (!createNewFile()) {
            Serial.println("  SD: Failed to create new file");
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
    
    if (samplesRead > 0) {
        Serial.print("  SD: Read ");
        Serial.print(samplesRead);
        Serial.println(" samples from ring buffer");
    }
    
    // Process each sample
    size_t samplesWritten = 0;
    for (size_t i = 0; i < samplesRead; i++) {
        // Format as CSV
        std::string line = sdSampleBuffer[i].toCSV() + "\n";
        
        // Add to the write buffer
        if (addLineToBuffer(line)) {
            samplesWritten++;
        } else {
            Serial.println("  SD: Failed to add line to buffer");
        }
    }
    
    // Check if buffer is near full and should be flushed
    if (writeBufferPos_ >= adc::SD_BLOCK_SIZE * 0.9) {
        Serial.println("  SD: Buffer nearly full, flushing");
        flushBuffer();
    }
    
    if (samplesWritten > 0) {
        Serial.print("  SD: Wrote ");
        Serial.print(samplesWritten);
        Serial.println(" samples to buffer");
    }
    
    return samplesWritten;
}

bool SDWriter::createNewFile(bool addHeader) {
    // Close any open file
    closeFile();
    
    // Generate a filename
    currentFilename_ = generateFilename();
    
    Serial.print("  SD: Creating new file: ");
    Serial.println(currentFilename_.c_str());
    
    // Create the file
    if (!dataFile_.open(currentFilename_.c_str(), O_WRONLY | O_CREAT | O_TRUNC)) {
        Serial.println("  SD: Failed to create file");
        return false;
    }
    
    // Reset counters
    fileCreationTime_ = millis();
    bytesWritten_ = 0;
    writeBufferPos_ = 0;
    
    // Write header if requested
    if (addHeader) {
        Serial.println("  SD: Writing header");
        if (!writeHeader()) {
            Serial.println("  SD: Failed to write header");
            closeFile();
            return false;
        }
    }
    
    Serial.println("  SD: File created successfully");
    return true;
}

bool SDWriter::closeFile() {
    // Flush any remaining data
    if (writeBufferPos_ > 0) {
        Serial.println("  SD: Flushing remaining data before closing");
        flushBuffer();
    }
    
    // Close the file
    if (dataFile_.isOpen()) {
        Serial.println("  SD: Closing file");
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
    
    Serial.print("  SD: Writing header: ");
    Serial.println(header.c_str());
    
    // Write to buffer
    return addLineToBuffer(header);
}

bool SDWriter::flushBuffer() {
    // Check if there's anything to write
    if (writeBufferPos_ == 0) {
        return true;
    }
    
    Serial.print("  SD: Flushing ");
    Serial.print(writeBufferPos_);
    Serial.println(" bytes to SD card");
    
    // Disable interrupts during SD write to prevent corruption
    noInterrupts();
    
    // Write the buffer to the file
    size_t bytesWritten = dataFile_.write(sdWriterBuffer, writeBufferPos_);
    
    // Re-enable interrupts
    interrupts();
    
    if (bytesWritten != writeBufferPos_) {
        Serial.print("  SD: Write error! Expected to write ");
        Serial.print(writeBufferPos_);
        Serial.print(" bytes but wrote ");
        Serial.println(bytesWritten);
    }
    
    // Update counters
    bytesWritten_ += bytesWritten;
    writeBufferPos_ = 0;
    
    // Sync file occasionally to prevent data loss on power failure
    if (bytesWritten_ % (adc::SD_BLOCK_SIZE * 10) == 0) {
        Serial.println("  SD: Syncing file to prevent data loss");
        dataFile_.sync();
    }
    
    return (bytesWritten == writeBufferPos_);
}

bool SDWriter::addLineToBuffer(const std::string& line) {
    // Check if line will fit in the buffer
    if (writeBufferPos_ + line.length() > adc::SD_BLOCK_SIZE) {
        // Buffer is full, flush it first
        Serial.println("  SD: Buffer full, flushing before adding line");
        if (!flushBuffer()) {
            Serial.println("  SD: Failed to flush buffer");
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