#include "storage/sd_thread.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace storage {

// Initialize static class members
SDWriter* SDThread::sdWriter_ = nullptr;
int SDThread::threadId_ = -1;
volatile bool SDThread::running_ = false;
uint32_t SDThread::totalWritten_ = 0;

bool SDThread::initialize(
    buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer,
    RingBuf<FsFile, config::SD_RING_BUF_CAPACITY>* sdRingBuf,
    uint8_t chipSelect) {
    
    util::Debug::info("SD Thread: Initializing");
    
    // Create the SD writer
    sdWriter_ = new SDWriter(ringBuffer, sdRingBuf);
    
    if (!sdWriter_) {
        util::Debug::error("SD Thread: Failed to create SD writer");
        return false;
    }
    
    // Initialize SD card
    if (!sdWriter_->begin(chipSelect)) {
        util::Debug::error("SD Thread: SD card initialization failed");
        return false;
    }
    
    util::Debug::info("SD Thread: Initialization successful");
    return true;
}

int SDThread::start() {
    // Check if already running
    if (running_ || threadId_ > 0) {
        util::Debug::warning("SD Thread: Already running with ID " + String(threadId_));
        return threadId_;
    }
    
    // Check if SD writer is initialized
    if (!sdWriter_) {
        util::Debug::error("SD Thread: SD writer not initialized");
        return -1;
    }
    
    // Reset counters
    totalWritten_ = 0;
    
    // Create the thread
    running_ = true;
    threadId_ = threads.addThread(threadFunction, 0, config::SD_WRITER_THREAD_STACK_SIZE);
    
    if (threadId_ <= 0) {
        util::Debug::error("SD Thread: Failed to create thread");
        running_ = false;
        return -2;
    }
    
    // Set thread priority
    threads.setTimeSlice(threadId_, config::SD_WRITER_THREAD_PRIORITY);
    
    util::Debug::info("SD Thread: Started with ID " + String(threadId_));
    return threadId_;
}

bool SDThread::stop() {
    if (!running_ || threadId_ <= 0) {
        return false;
    }
    
    // Signal thread to stop
    running_ = false;
    
    // Wait for thread to finish
    threads.wait(threadId_, 1000);  // Wait up to 1 second
    threadId_ = -1;
    
    // Close the file
    if (sdWriter_) {
        sdWriter_->closeFile();
    }
    
    util::Debug::info("SD Thread: Stopped");
    return true;
}

bool SDThread::isRunning() {
    return running_ && threadId_ > 0;
}

SDWriter* SDThread::getWriter() {
    return sdWriter_;
}

void SDThread::setChannelConfigs(const std::vector<adc::ChannelConfig>& channelConfigs) {
    if (!sdWriter_) {
        util::Debug::error("SD Thread: SD writer not initialized");
        return;
    }
    
    sdWriter_->setChannelNames(channelConfigs);
}

bool SDThread::flush() {
    if (!sdWriter_) {
        return false;
    }
    
    return sdWriter_->flush();
}

void SDThread::threadFunction(void* arg) {
    util::Debug::info("SD Thread: Thread started");
    
    // Create a new data file
    if (sdWriter_ && !sdWriter_->createNewFile()) {
        util::Debug::error("SD Thread: Failed to create initial data file!");
    } else if (sdWriter_) {
        util::Debug::info("SD Thread: Created initial file: " + 
            String(sdWriter_->getCurrentFilename().c_str()));
    }
    
    // Main SD writer loop
    uint32_t lastStatusTime = 0;
    
    while (running_) {
        // Process samples and write to SD card
        if (sdWriter_) {
            size_t samplesWritten = sdWriter_->process();
            totalWritten_ += samplesWritten;
            
            // Print status occasionally
            uint32_t now = millis();
            if (now - lastStatusTime > 30000) {
                util::Debug::info("SD Thread: Samples written: " + 
                                String(totalWritten_) + 
                                ", File: " + 
                                String(sdWriter_->getCurrentFilename().c_str()) + 
                                ", Bytes: " + 
                                String(sdWriter_->getBytesWritten()));
                lastStatusTime = now;
            }
            
            // Check health and attempt recovery if needed
            if (!sdWriter_->isHealthy()) {
                util::Debug::warning("SD Thread: Detected unhealthy state, attempting recovery");
                sdWriter_->closeFile();
                delay(100);
                
                if (sdWriter_->createNewFile()) {
                    util::Debug::info("SD Thread: Recovery successful");
                    sdWriter_->resetHealth();
                } else {
                    util::Debug::warning("SD Thread: Recovery failed, will retry later");
                    delay(5000);
                }
            }
        }
        
        // Small yield to allow other threads to run
        threads.yield();
    }
    
    // Close the file before exiting
    if (sdWriter_) {
        sdWriter_->closeFile();
    }
    
    util::Debug::info("SD Thread: Thread ended");
}

} // namespace storage
} // namespace baja