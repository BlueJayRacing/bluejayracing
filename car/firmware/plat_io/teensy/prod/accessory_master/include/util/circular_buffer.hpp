#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cstring>

namespace baja {
namespace buffer {

/**
 * @brief Lock-free circular buffer with overwrite policy for fast paths
 * 
 * This lightweight circular buffer is designed for high-speed operation
 * where old data can be overwritten when the buffer is full. It is not
 * protected by mutexes and is intended for single-producer, single-consumer
 * scenarios.
 * 
 * @tparam T Type of items stored in the buffer
 * @tparam SIZE Size of the buffer in number of elements
 */
template<typename T, size_t SIZE>
class CircularBuffer {
public:
    /**
     * @brief Construct a new CircularBuffer with external storage
     * 
     * @param externalBuffer Pointer to externally allocated buffer in DMAMEM
     */
    CircularBuffer(T* externalBuffer) : 
        buffer_(externalBuffer),
        readIndex_(0), 
        writeIndex_(0),
        fullFlag_(false),
        totalWriteCount_(0),
        totalReadCount_(0),
        overwriteCount_(0) {
        
        Serial.println("CircularBuffer initialized with capacity " + String(SIZE));
    }

    /**
     * @brief Write an item to the buffer, overwriting oldest item if full
     * 
     * This function is designed to be non-blocking and fast. If the buffer
     * is full, it will overwrite the oldest data.
     * 
     * @param item Item to write to the buffer
     * @return true Always returns true as writes never fail
     */
    bool write(const T& item) {
        // Store item at current write position
        buffer_[writeIndex_] = item;
        
        // Move write index forward
        writeIndex_ = (writeIndex_ + 1) % SIZE;
        
        // If write index catches up to read index, set full flag and 
        // increment read index to indicate oldest data was overwritten
        if (writeIndex_ == readIndex_) {
            fullFlag_ = true;
            readIndex_ = (readIndex_ + 1) % SIZE;
            overwriteCount_++;
        }
        
        // Increment total write count
        totalWriteCount_++;
        
        return true;
    }

    /**
     * @brief Read an item from the buffer
     * 
     * @param item Output parameter where read item will be stored
     * @return true if read was successful, false if buffer is empty
     */
    bool read(T& item) {
        // Check if buffer is empty
        if (isEmpty()) {
            return false;
        }
        
        // Read item from current read position
        item = buffer_[readIndex_];
        
        // Move read index forward
        readIndex_ = (readIndex_ + 1) % SIZE;
        
        // If buffer was full before, it's not full anymore
        fullFlag_ = false;
        
        // Increment total read count
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
        // Check if buffer is empty or offset is too large
        if (isEmpty() || offset >= available()) {
            return false;
        }
        
        // Calculate peek index
        size_t peekIndex = (readIndex_ + offset) % SIZE;
        
        // Read item at peek index
        item = buffer_[peekIndex];
        
        return true;
    }

    /**
     * @brief Peek multiple items from the buffer
     * 
     * @param items Array to store peeked items
     * @param maxItems Maximum number of items to peek
     * @param startOffset Starting offset from read index
     * @return Number of items actually peeked
     */
    size_t peekMultiple(T* items, size_t maxItems, size_t startOffset = 0) {
        size_t avail = available();
        
        // Check if buffer is empty or offset is too large
        if (isEmpty() || startOffset >= avail) {
            return 0;
        }
        
        // Calculate how many items we can actually peek
        size_t itemsToPeek = (maxItems < (avail - startOffset)) ? 
                           maxItems : (avail - startOffset);
        
        // Peek each item
        for (size_t i = 0; i < itemsToPeek; i++) {
            size_t peekIndex = (readIndex_ + startOffset + i) % SIZE;
            items[i] = buffer_[peekIndex];
        }
        
        return itemsToPeek;
    }

    /**
     * @brief Read multiple items from the buffer
     * 
     * @param items Array to store read items
     * @param maxItems Maximum number of items to read
     * @return Number of items actually read
     */
    size_t readMultiple(T* items, size_t maxItems) {
        size_t avail = available();
        
        // Check if buffer is empty
        if (isEmpty()) {
            return 0;
        }
        
        // Calculate how many items we can actually read
        size_t itemsToRead = (maxItems < avail) ? maxItems : avail;
        
        // Read each item
        for (size_t i = 0; i < itemsToRead; i++) {
            items[i] = buffer_[readIndex_];
            readIndex_ = (readIndex_ + 1) % SIZE;
        }
        
        // If buffer was full before, it's not full anymore
        if (itemsToRead > 0) {
            fullFlag_ = false;
        }
        
        // Update total read count
        totalReadCount_ += itemsToRead;
        
        return itemsToRead;
    }

    /**
     * @brief Check if buffer is empty
     * 
     * @return true if buffer is empty, false otherwise
     */
    bool isEmpty() const {
        return !fullFlag_ && (readIndex_ == writeIndex_);
    }

    /**
     * @brief Check if buffer is full
     * 
     * @return true if buffer is full, false otherwise
     */
    bool isFull() const {
        return fullFlag_;
    }

    /**
     * @brief Get number of items available to read
     * 
     * @return Number of items in the buffer
     */
    size_t available() const {
        if (isEmpty()) {
            return 0;
        }
        
        if (isFull()) {
            return SIZE;
        }
        
        if (writeIndex_ > readIndex_) {
            return writeIndex_ - readIndex_;
        } else {
            return SIZE - (readIndex_ - writeIndex_);
        }
    }

    /**
     * @brief Get number of free spaces in the buffer
     * 
     * @return Number of free spaces
     */
    size_t free() const {
        return SIZE - available();
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
        readIndex_ = 0;
        writeIndex_ = 0;
        fullFlag_ = false;
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
     * @brief Get number of buffer overwrites (writes when buffer was full)
     * 
     * @return Overwrite count
     */
    uint64_t getOverwriteCount() const {
        return overwriteCount_;
    }
    
    /**
     * @brief Print buffer statistics
     */
    void printStats() const {
        Serial.println("CircularBuffer Statistics:");
        Serial.print("  Capacity: ");
        Serial.println(SIZE);
        Serial.print("  Current usage: ");
        Serial.print(available());
        Serial.print("/");
        Serial.print(SIZE);
        Serial.print(" (");
        Serial.print((float)available() / SIZE * 100.0f);
        Serial.println("%)");
        Serial.print("  Total writes: ");
        Serial.println(totalWriteCount_);
        Serial.print("  Total reads: ");
        Serial.println(totalReadCount_);
        Serial.print("  Overwrites: ");
        Serial.println(overwriteCount_);
    }

private:
    // External buffer pointer (stored in DMAMEM)
    T* buffer_;
    
    // Buffer state variables (volatile for thread safety)
    volatile size_t readIndex_;
    volatile size_t writeIndex_;
    volatile bool fullFlag_;
    
    // Statistics
    volatile uint64_t totalWriteCount_;
    volatile uint64_t totalReadCount_;
    volatile uint64_t overwriteCount_;
};

} // namespace buffer
} // namespace baja