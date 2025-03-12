#pragma once

#include <Arduino.h>
#include <TeensyThreads.h>
#include <cstdint>
#include <cstring>

namespace baja {
namespace buffer {

/**
 * @brief Thread-safe ring buffer implementation optimized for Teensy 4.1
 * 
 * This lightweight ring buffer uses an externally allocated array in DMAMEM,
 * keeping only control variables in RAM1 to save memory.
 * 
 * @tparam T Type of items stored in the buffer
 * @tparam SIZE Size of the buffer in number of elements
 */
template<typename T, size_t SIZE>
class RingBuffer {
public:
    /**
     * @brief Construct a new Ring Buffer object with external storage
     * 
     * @param externalBuffer Pointer to externally allocated buffer in DMAMEM
     */
    RingBuffer(T* externalBuffer) : 
        buffer_(externalBuffer),
        readIndex_(0), 
        writeIndex_(0), 
        count_(0),
        totalWriteCount_(0),
        totalReadCount_(0),
        overrunCount_(0) {
        Serial.println("RingBuffer initialized with capacity " + String(SIZE));
    }

    /**
     * @brief Write an item to the buffer (for ISR usage)
     * 
     * @param item Item to write to the buffer
     * @return true if write was successful, false if buffer is full
     */
    bool writeFromISR(const T& item) {
        bool success = false;
        
        // Critical section - no other thread can modify buffer during this time
        if (count_ < SIZE) {
            buffer_[writeIndex_] = item;
            writeIndex_ = (writeIndex_ + 1) % SIZE;
            count_++;
            totalWriteCount_++;
            success = true;
        } else {
            // Buffer is full
            overrunCount_++;
        }
        
        return success;
    }

    /**
     * @brief Write an item to the buffer (for thread usage)
     * 
     * @param item Item to write to the buffer
     * @return true if write was successful, false if buffer is full
     */
    bool write(const T& item) {
        Threads::Scope lock(mutex_);
        
        if (count_ >= SIZE) {
            overrunCount_++;
            return false;
        }
        
        buffer_[writeIndex_] = item;
        writeIndex_ = (writeIndex_ + 1) % SIZE;
        count_++;
        totalWriteCount_++;
        return true;
    }

    /**
     * @brief Read an item from the buffer (removing it)
     * 
     * @param item Output parameter where read item will be stored
     * @return true if read was successful, false if buffer is empty
     */
    bool read(T& item) {
        Threads::Scope lock(mutex_);
        
        if (count_ == 0) {
            return false;
        }
        
        item = buffer_[readIndex_];
        readIndex_ = (readIndex_ + 1) % SIZE;
        count_--;
        totalReadCount_++;
        return true;
    }

    /**
     * @brief Peek at an item in the buffer without removing it
     * 
     * @param item Output parameter where peeked item will be stored
     * @param offset How many items ahead to peek (0 = next item to be read)
     * @return true if peek was successful, false if offset is beyond available items
     */
    bool peek(T& item, size_t offset = 0) const {
        Threads::Scope lock(mutex_);
        
        if (offset >= count_) {
            return false;
        }
        
        size_t peekIndex = (readIndex_ + offset) % SIZE;
        item = buffer_[peekIndex];
        return true;
    }

    /**
     * @brief Read multiple items into a buffer
     * 
     * @param items Array to store read items
     * @param maxItems Maximum number of items to read
     * @return Number of items actually read
     */
    size_t readMultiple(T* items, size_t maxItems) {
        Threads::Scope lock(mutex_);
        
        size_t itemsToRead = (maxItems < count_) ? maxItems : count_;
        
        for (size_t i = 0; i < itemsToRead; i++) {
            items[i] = buffer_[readIndex_];
            readIndex_ = (readIndex_ + 1) % SIZE;
        }
        
        count_ -= itemsToRead;
        totalReadCount_ += itemsToRead;
        
        // Only log larger reads to reduce log spam
        if (itemsToRead > 1000) {
            Serial.print("RingBuffer: Read ");
            Serial.print(itemsToRead);
            Serial.print(" samples. Total reads=");
            Serial.print(totalReadCount_);
            Serial.print(", remaining=");
            Serial.println(count_);
        }
        
        return itemsToRead;
    }

    /**
     * @brief Get number of items available to read
     * 
     * @return Number of items in the buffer
     */
    size_t available() const {
        return count_;
    }

    /**
     * @brief Get number of free spaces in the buffer
     * 
     * @return Number of free spaces
     */
    size_t free() const {
        return SIZE - count_;
    }

    /**
     * @brief Get the buffer capacity
     * 
     * @return Total buffer capacity (SIZE)
     */
    constexpr size_t capacity() const {
        return SIZE;
    }

    /**
     * @brief Clear the buffer
     */
    void clear() {
        Threads::Scope lock(mutex_);
        readIndex_ = 0;
        writeIndex_ = 0;
        count_ = 0;
    }
    
    /**
     * @brief Get total number of items written to the buffer since creation
     * 
     * @return Total write count
     */
    uint64_t getTotalWriteCount() const {
        return totalWriteCount_;
    }
    
    /**
     * @brief Get total number of items read from the buffer since creation
     * 
     * @return Total read count
     */
    uint64_t getTotalReadCount() const {
        return totalReadCount_;
    }
    
    /**
     * @brief Get number of buffer overruns (writes when buffer was full)
     * 
     * @return Overrun count
     */
    uint64_t getOverrunCount() const {
        return overrunCount_;
    }
    
    /**
     * @brief Print buffer statistics
     */
    void printStats() const {
        Threads::Scope lock(mutex_);
        
        Serial.println("RingBuffer Statistics:");
        Serial.print("  Capacity: ");
        Serial.println(SIZE);
        Serial.print("  Current usage: ");
        Serial.print(count_);
        Serial.print("/");
        Serial.print(SIZE);
        Serial.print(" (");
        Serial.print((float)count_ / SIZE * 100.0f);
        Serial.println("%)");
        Serial.print("  Total writes: ");
        Serial.println(totalWriteCount_);
        Serial.print("  Total reads: ");
        Serial.println(totalReadCount_);
        Serial.print("  Overruns: ");
        Serial.println(overrunCount_);
    }

private:
    // External buffer pointer (stored in DMAMEM)
    T* buffer_;
    
    // Buffer state variables
    volatile size_t readIndex_;
    volatile size_t writeIndex_;
    volatile size_t count_;
    
    // Statistics
    volatile uint64_t totalWriteCount_;
    volatile uint64_t totalReadCount_;
    volatile uint64_t overrunCount_;
    
    // Mutex for thread synchronization
    mutable Threads::Mutex mutex_;
};

} // namespace buffer
} // namespace baja