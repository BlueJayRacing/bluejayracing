#pragma once

#include <cstdint>
#include <cstddef>

namespace baja {

/**
 * @brief Global configuration settings for the application
 */
namespace config {

// ADC Configuration
constexpr uint8_t ADC_CHANNEL_COUNT = 16;  // Total number of ADC channels
constexpr int ADC_DEFAULT_GAIN = 1;        // Default gain for ADC channels
constexpr bool ADC_ENABLE_ALL_CHANNELS = false; // Enable all 16 channels

// Buffer Configuration
constexpr size_t SAMPLE_RING_BUFFER_SIZE = 480000;  // Size of the sample ring buffer in samples
constexpr float DATA_BUFFER_WRITE_THRESHOLD = 0.10f; // Start writing at 10% data buffer utilization
constexpr size_t MIN_BYTES_FOR_WRITE = 512;       // Minimum bytes to write (1 sector)

// SD Card Configuration
constexpr size_t SD_SECTOR_SIZE = 512;              // SD card sector size in bytes
constexpr size_t SD_RING_BUF_CAPACITY = 480 * SD_SECTOR_SIZE;  // ~245KB in EXTMEM (480 sectors)
constexpr size_t SD_PREALLOC_SIZE = 5UL * 1024 * 1024; // 5MB file preallocation (reduced from 50MB)
constexpr uint32_t SD_FILE_ROTATION_INTERVAL_MS = 30 * 1000; // 30 seconds for testing (adjust as needed)
constexpr size_t SD_MAX_FILENAME_LENGTH = 32;

// Thread Configuration
constexpr int THREAD_SLICE_MICROS = 50;   // Time slice in microseconds for TeensyThreads
constexpr int SD_WRITER_THREAD_PRIORITY = 10; // Higher number = higher priority (for TeensyThreads)
constexpr int SD_WRITER_THREAD_STACK_SIZE = 8192; // Stack size for SD writer thread

// Protocol Buffer Configuration
constexpr size_t PB_MESSAGE_BUFFER_SIZE = 128;  // Increased from 64 to 128
constexpr size_t PB_MAX_MESSAGE_SIZE = 1472;    // Increased from 3072 to 4096
constexpr uint16_t PB_MAX_TIMESTAMP_DELTA = 65000; // Maximum timestamp delta for fixed chunks
constexpr size_t MAX_SAMPLES_PER_BATCH = 50;   // Reduced from 150 to 120 to keep messages smaller
constexpr uint32_t PB_SERIALIZATION_INTERVAL_MS = 200; // Increased from 50 to 100ms
constexpr bool USE_VERBOSE_DATA_CHUNK = true;   // Use VerboseDataChunk instead of FixedDataChunk
constexpr bool PB_DEBUG_LOGGING = true;         // Enable detailed logging for protobuf operations

constexpr bool USE_HARD_CODED_ENCODING = true;


// Debug configuration
constexpr bool ENABLE_DETAILED_LOGS = true; // Enable detailed logging

} // namespace config
} // namespace baja