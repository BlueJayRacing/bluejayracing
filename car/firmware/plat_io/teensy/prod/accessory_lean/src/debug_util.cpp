#include "debug_util.hpp"
#include <stdio.h>
#include <stdarg.h>

namespace baja {
namespace util {

// Initialize static members
DebugOutput::VerbosityLevel DebugOutput::currentLevel = DebugOutput::INFO;
Stream* DebugOutput::serial = &Serial;

void DebugOutput::init(VerbosityLevel level, Stream* serialPtr) {
    currentLevel = level;
    if (serialPtr != nullptr) {
        serial = serialPtr;
    }
}

void DebugOutput::setLevel(VerbosityLevel level) {
    currentLevel = level;
}

DebugOutput::VerbosityLevel DebugOutput::getLevel() {
    return currentLevel;
}

void DebugOutput::error(const char* msg, bool newLine) {
    print(ERROR, msg, newLine);
}

void DebugOutput::error(const std::string& msg, bool newLine) {
    print(ERROR, msg, newLine);
}

void DebugOutput::error(const String& msg, bool newLine) {
    print(ERROR, msg.c_str(), newLine);
}

void DebugOutput::warning(const char* msg, bool newLine) {
    print(WARNING, msg, newLine);
}

void DebugOutput::warning(const std::string& msg, bool newLine) {
    print(WARNING, msg, newLine);
}

void DebugOutput::warning(const String& msg, bool newLine) {
    print(WARNING, msg.c_str(), newLine);
}

void DebugOutput::info(const char* msg, bool newLine) {
    print(INFO, msg, newLine);
}

void DebugOutput::info(const std::string& msg, bool newLine) {
    print(INFO, msg, newLine);
}

void DebugOutput::info(const String& msg, bool newLine) {
    print(INFO, msg.c_str(), newLine);
}

void DebugOutput::detail(const char* msg, bool newLine) {
    print(DETAIL, msg, newLine);
}

void DebugOutput::detail(const std::string& msg, bool newLine) {
    print(DETAIL, msg, newLine);
}

void DebugOutput::detail(const String& msg, bool newLine) {
    print(DETAIL, msg.c_str(), newLine);
}

void DebugOutput::debug(const char* msg, bool newLine) {
    print(DEBUG, msg, newLine);
}

void DebugOutput::debug(const std::string& msg, bool newLine) {
    print(DEBUG, msg, newLine);
}

void DebugOutput::debug(const String& msg, bool newLine) {
    print(DEBUG, msg.c_str(), newLine);
}

void DebugOutput::print(VerbosityLevel level, const char* msg, bool newLine) {
    if (level <= currentLevel) {
        // Print prefix based on level
        switch (level) {
            case ERROR:
                serial->print("[ERROR] ");
                break;
            case WARNING:
                serial->print("[WARN] ");
                break;
            case INFO:
                serial->print("[INFO] ");
                break;
            case DETAIL:
                serial->print("[DETAIL] ");
                break;
            case DEBUG:
                serial->print("[DEBUG] ");
                break;
            default:
                break;
        }
        
        // Print message
        serial->print(msg);
        
        // Add newline if requested
        if (newLine) {
            serial->println();
        }
    }
}

void DebugOutput::print(VerbosityLevel level, const std::string& msg, bool newLine) {
    print(level, msg.c_str(), newLine);
}

void DebugOutput::print(VerbosityLevel level, const String& msg, bool newLine) {
    print(level, msg.c_str(), newLine);
}

void DebugOutput::printf(VerbosityLevel level, const char* format, ...) {
    if (level <= currentLevel) {
        char buffer[256]; // Buffer for formatted string
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        // Print with appropriate prefix
        print(level, buffer);
    }
}

bool DebugOutput::isEnabled(VerbosityLevel level) {
    return level <= currentLevel;
}

void DebugOutput::printBuffer(VerbosityLevel level, const uint8_t* buffer, 
                             size_t length, uint8_t bytesPerLine) {
    if (level <= currentLevel) {
        char lineBuf[128];
        char hexBuf[8];
        
        for (size_t i = 0; i < length; i += bytesPerLine) {
            // Start with address
            snprintf(lineBuf, sizeof(lineBuf), "%04X: ", (unsigned int)i);
            
            // Add hex bytes
            size_t lineLength = min(bytesPerLine, length - i);
            for (size_t j = 0; j < lineLength; j++) {
                snprintf(hexBuf, sizeof(hexBuf), "%02X ", buffer[i + j]);
                strcat(lineBuf, hexBuf);
            }
            
            // Pad with spaces if line is incomplete
            for (size_t j = lineLength; j < bytesPerLine; j++) {
                strcat(lineBuf, "   ");
            }
            
            // Add ASCII representation
            strcat(lineBuf, " | ");
            for (size_t j = 0; j < lineLength; j++) {
                char c = buffer[i + j];
                if (c >= 32 && c <= 126) { // Printable ASCII
                    snprintf(hexBuf, sizeof(hexBuf), "%c", c);
                } else {
                    snprintf(hexBuf, sizeof(hexBuf), ".");
                }
                strcat(lineBuf, hexBuf);
            }
            
            // Print the line
            print(level, lineBuf);
        }
    }
}

} // namespace util
} // namespace baja