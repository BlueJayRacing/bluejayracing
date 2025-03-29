#pragma once

#include <cstdint>
#include <cstddef>

namespace baja {

/**
 * @brief Global configuration settings for the application
 */
namespace config {

// Buffer Configuration
constexpr size_t SAMPLE_RING_BUFFER_SIZE = 480000;  // Size of the sample ring buffer in samples
constexpr size_t FAST_BUFFER_SIZE = 4096; // Size of the fast path buffer for network transmission
constexpr float DATA_BUFFER_WRITE_THRESHOLD = 0.10f; // Start writing at 10% data buffer utilization
constexpr size_t MIN_BYTES_FOR_WRITE = 512;       // Minimum bytes to write (1 sector)

// Fast path buffer downsampling
constexpr uint8_t FAST_BUFFER_DOWNSAMPLE_RATIO = 1; // Only send 1 in N samples to fast buffer per channel

// SD Card Configuration
constexpr size_t SD_SECTOR_SIZE = 512;              // SD card sector size in bytes
constexpr size_t SD_RING_BUF_CAPACITY = 480 * SD_SECTOR_SIZE;  // ~245KB in EXTMEM (480 sectors)
constexpr size_t SD_PREALLOC_SIZE = 50UL * 1024 * 1024; // 50MB file preallocation
constexpr uint32_t SD_FILE_ROTATION_INTERVAL_MS = 120 * 1000; // 30 seconds for testing (adjust as needed)
constexpr size_t SD_MAX_FILENAME_LENGTH = 32;
constexpr bool CUSTOM_STRING_CONVERSION_ROUTINE = true; // Use custom string conversion routine
constexpr uint32_t SD_SYNC_INTERVAL_MS = 15 * 1000;     // 15 seconds periodic sync
constexpr uint32_t SD_MAX_SYNC_TIME_US = 60;           // Warning threshold for sync time
constexpr size_t SD_SYNC_MIN_BUFFER_BYTES = 16 * 1024;  // Minimum bytes before doing periodic sync
constexpr bool CSV_INCLUDE_CHANNEL_NAMES = false;  // Include channel names in CSV for readability

// Protocol Buffer Configuration
constexpr size_t PB_MAX_MESSAGE_SIZE = 1472;   // Maximum size of ethernet frame minus UDP Header
constexpr size_t FIXED_SAMPLE_COUNT = 50;   // Maximum samples to process per batch
constexpr bool USE_HARD_CODED_ENCODING = true;  // Use hard-coded optimized encoders

// ADC settings
constexpr double ADC_DEFAULT_GAIN = 1.0;          // Default gain for ADC channels
constexpr bool ADC_ENABLE_ALL_CHANNELS = false;   // Enable all channels regardless of defaults

} // namespace config
} // namespace baja