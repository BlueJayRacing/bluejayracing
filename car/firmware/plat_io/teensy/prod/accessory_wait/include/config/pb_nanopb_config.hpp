#pragma once

// Nanopb configuration options

// Uncomment to use dynamic allocation instead of fixed-size arrays
// #define PB_ENABLE_MALLOC 1

// Maximum recursion depth for message fields
#define PB_MAX_RECURSION_DEPTH 20

// Buffer size for strings
#define PB_FIELD_TEXTBUF_WIDTH 256

// Set to 1 to have a zero-argument form of PB_STATIC_ASSERT
#define PB_STATIC_ASSERT_BOOL 1

// 64-bit integer support
#define PB_WITHOUT_64BIT 0

// Optional fields support
#define PB_NO_ERRMSG 1  // Don't include error messages to save space

// We don't need callback support
#define PB_FIELD_32BIT 0

// Advanced support for fixed point arithmetic
#define PB_CONVERT_DOUBLE_FLOAT 1