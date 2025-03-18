#pragma once

#include <Arduino.h>
#include <string>

namespace baja {
namespace util {

/**
 * @brief Utility class for debug output
 * 
 * Provides consistent debug output across the application
 * with controllable verbosity levels.
 */
class DebugOutput {
public:
    // Debug verbosity levels
    enum VerbosityLevel {
        SILENT = 0,      // No output
        ERROR = 1,       // Critical errors only
        WARNING = 2,     // Warnings and errors
        INFO = 3,        // General information
        DETAIL = 4,      // Detailed information
        DEBUG = 5        // Debug diagnostics
    };

    /**
     * @brief Initialize the debug output
     * 
     * @param level Initial verbosity level
     * @param serialPtr Pointer to the Serial object (defaults to &Serial)
     */
    static void init(VerbosityLevel level = INFO, Stream* serialPtr = nullptr);

    /**
     * @brief Set the verbosity level
     * 
     * @param level New verbosity level
     */
    static void setLevel(VerbosityLevel level);

    /**
     * @brief Get the current verbosity level
     * 
     * @return Current verbosity level
     */
    static VerbosityLevel getLevel();

    /**
     * @brief Print error message
     * 
     * @param msg Message to print
     * @param newLine Add newline (default: true)
     */
    static void error(const char* msg, bool newLine = true);
    static void error(const std::string& msg, bool newLine = true);
    static void error(const String& msg, bool newLine = true);

    /**
     * @brief Print warning message
     * 
     * @param msg Message to print
     * @param newLine Add newline (default: true)
     */
    static void warning(const char* msg, bool newLine = true);
    static void warning(const std::string& msg, bool newLine = true);
    static void warning(const String& msg, bool newLine = true);

    /**
     * @brief Print info message
     * 
     * @param msg Message to print
     * @param newLine Add newline (default: true)
     */
    static void info(const char* msg, bool newLine = true);
    static void info(const std::string& msg, bool newLine = true);
    static void info(const String& msg, bool newLine = true);

    /**
     * @brief Print detailed message
     * 
     * @param msg Message to print
     * @param newLine Add newline (default: true)
     */
    static void detail(const char* msg, bool newLine = true);
    static void detail(const std::string& msg, bool newLine = true);
    static void detail(const String& msg, bool newLine = true);

    /**
     * @brief Print debug message
     * 
     * @param msg Message to print
     * @param newLine Add newline (default: true)
     */
    static void debug(const char* msg, bool newLine = true);
    static void debug(const std::string& msg, bool newLine = true);
    static void debug(const String& msg, bool newLine = true);

    /**
     * @brief Print message at specified level
     * 
     * @param level Verbosity level
     * @param msg Message to print
     * @param newLine Add newline (default: true)
     */
    static void print(VerbosityLevel level, const char* msg, bool newLine = true);
    static void print(VerbosityLevel level, const std::string& msg, bool newLine = true);
    static void print(VerbosityLevel level, const String& msg, bool newLine = true);

    /**
     * @brief Print message with formatting (like printf)
     * 
     * @param level Verbosity level
     * @param format Format string
     * @param ... Variable arguments
     */
    static void printf(VerbosityLevel level, const char* format, ...);

    /**
     * @brief Check if a message at the given level would be printed
     * 
     * @param level Verbosity level to check
     * @return true if messages at this level are being printed
     */
    static bool isEnabled(VerbosityLevel level);

    /**
     * @brief Print buffer contents in hex format
     * 
     * @param level Verbosity level
     * @param buffer Pointer to buffer
     * @param length Length of buffer
     * @param bytesPerLine Number of bytes per line (default: 16)
     */
    static void printBuffer(VerbosityLevel level, const uint8_t* buffer, 
                           size_t length, uint8_t bytesPerLine = 16);

private:
    static VerbosityLevel currentLevel;
    static Stream* serial;
};

// Shorthand aliases for DebugOutput class
using Debug = DebugOutput;

} // namespace util
} // namespace baja