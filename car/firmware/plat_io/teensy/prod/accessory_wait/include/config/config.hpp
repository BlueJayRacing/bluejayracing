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
constexpr size_t FAST_BUFFER_SIZE = 1024; // Size of the fast path buffer for network transmission
constexpr float DATA_BUFFER_WRITE_THRESHOLD = 0.10f; // Start writing at 10% data buffer utilization
constexpr size_t MIN_BYTES_FOR_WRITE = 512;       // Minimum bytes to write (1 sector)

// Fast path buffer downsampling
constexpr uint8_t FAST_BUFFER_DOWNSAMPLE_RATIO = 10; // Only send 1 in N samples to fast buffer per channel

// SD Card Configuration
constexpr size_t SD_SECTOR_SIZE = 512;              // SD card sector size in bytes
constexpr size_t SD_RING_BUF_CAPACITY = 480 * SD_SECTOR_SIZE;  // ~245KB in EXTMEM (480 sectors)
constexpr size_t SD_PREALLOC_SIZE = 50UL * 1024 * 1024; // 5MB file preallocation (reduced from 50MB)
constexpr uint32_t SD_FILE_ROTATION_INTERVAL_MS = 90 * 1000; // 30 seconds for testing (adjust as needed)
constexpr size_t SD_MAX_FILENAME_LENGTH = 32;

// Thread Configuration
constexpr int THREAD_SLICE_MICROS = 10;   // Time slice in microseconds for TeensyThreads
constexpr int SD_WRITER_THREAD_PRIORITY = 5; // Higher number = higher priority
constexpr int ADC_THREAD_PRIORITY = 15;      // High priority for ADC thread
constexpr int PBUDP_THREAD_PRIORITY = 6;    // Medium-high priority for combined PB+UDP thread
constexpr int SD_WRITER_THREAD_STACK_SIZE = 8192; // Stack size for SD writer thread
constexpr int PBUDP_THREAD_STACK_SIZE = 108192;    // Stack size for combined PB+UDP thread

// Protocol Buffer Configuration
constexpr size_t PB_MESSAGE_BUFFER_SIZE = 32;  // Size of the intermediate encoded message buffer
constexpr size_t PB_MAX_MESSAGE_SIZE = 1472;   // Maximum size of ethernet frame minus UDP Header
constexpr uint16_t PB_MAX_TIMESTAMP_DELTA = 65000; // Maximum timestamp delta for fixed chunks
constexpr size_t FIXED_SAMPLE_COUNT = 50;   // Maximum samples to process per batch
constexpr uint32_t PB_SERIALIZATION_INTERVAL_MS = 50; // Process at 20Hz rate for low latency
constexpr bool USE_VERBOSE_DATA_CHUNK = true;   // Use verbose encoding format
constexpr bool USE_HARD_CODED_ENCODING = false;  // Use hard-coded optimized encoders
constexpr bool PB_DEBUG_LOGGING = true;         // Enable detailed logging for protobuf operations

// Combined PBUDP Thread Configuration
constexpr uint32_t PBUDP_MIN_INTERVAL_MS = 50;   // Minimum interval between transmissions (max 20Hz)
constexpr uint32_t PBUDP_TARGET_LATENCY_MS = 50; // Target latency for samples to reach network

// Debug configuration
constexpr bool ENABLE_DETAILED_LOGS = true; // Enable detailed logging

} // namespace config
} // namespace baja