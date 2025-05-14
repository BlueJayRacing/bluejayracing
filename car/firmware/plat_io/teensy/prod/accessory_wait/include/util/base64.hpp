#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cstring>

namespace baja {
namespace util {

/**
 * @brief Base64 encoding/decoding utilities
 */
class Base64 {
public:
    /**
     * @brief Calculate the encoded size for a given input size
     * 
     * @param inputSize Size of the data to encode
     * @return Size of the encoded data (including null terminator)
     */
    static size_t encodedSize(size_t inputSize) {
        return ((inputSize + 2) / 3) * 4 + 1; // +1 for null terminator
    }
    
    /**
     * @brief Calculate the decoded size for a given encoded size
     * 
     * @param encodedSize Size of the encoded data
     * @return Maximum size of the decoded data
     */
    static size_t decodedSize(size_t encodedSize) {
        return (encodedSize / 4) * 3;
    }
    
    /**
     * @brief Encode data to Base64
     * 
     * @param input Data to encode
     * @param inputSize Size of the data to encode
     * @param output Buffer to store the encoded data (must be at least encodedSize())
     * @return Size of the encoded data (excluding null terminator)
     */
    static size_t encode(const uint8_t* input, size_t inputSize, char* output) {
        if (!input || !output || inputSize == 0) {
            if (output) *output = '\0';
            return 0;
        }
        
        const char* base64_chars = 
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        
        size_t i = 0, j = 0;
        uint8_t char_array_3[3];
        uint8_t char_array_4[4];
        
        while (inputSize--) {
            char_array_3[i++] = *(input++);
            if (i == 3) {
                char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
                char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
                char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
                char_array_4[3] = char_array_3[2] & 0x3f;
                
                for (i = 0; i < 4; i++) {
                    output[j++] = base64_chars[char_array_4[i]];
                }
                i = 0;
            }
        }
        
        if (i) {
            for (size_t k = i; k < 3; k++) {
                char_array_3[k] = '\0';
            }
            
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            
            for (size_t k = 0; k < 4; k++) {
                if (k < i + 1) {
                    output[j++] = base64_chars[char_array_4[k]];
                } else {
                    output[j++] = '=';
                }
            }
        }
        
        output[j] = '\0';
        return j;
    }
    
    /**
     * @brief Decode Base64 data
     * 
     * @param input Base64 encoded data
     * @param output Buffer to store the decoded data (must be at least decodedSize())
     * @return Size of the decoded data
     */
    static size_t decode(const char* input, uint8_t* output) {
        if (!input || !output || !*input) {
            return 0;
        }
        
        size_t inputLen = strlen(input);
        size_t padding = 0;
        
        if (input[inputLen - 1] == '=') padding++;
        if (input[inputLen - 2] == '=') padding++;
        
        // Calculate output length
        size_t outputLen = ((inputLen + 3) / 4) * 3 - padding;
        
        size_t i = 0, j = 0;
        uint8_t char_array_4[4], char_array_3[3];
        size_t in_len = 0;
        
        while (inputLen-- && input[in_len] != '=' && is_base64(input[in_len])) {
            char_array_4[i++] = input[in_len++];
            if (i == 4) {
                for (i = 0; i < 4; i++) {
                    char_array_4[i] = base64_chars_index(char_array_4[i]);
                }
                
                char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
                char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
                char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
                
                for (i = 0; i < 3; i++) {
                    output[j++] = char_array_3[i];
                }
                i = 0;
            }
        }
        
        if (i) {
            for (size_t k = i; k < 4; k++) {
                char_array_4[k] = 0;
            }
            
            for (size_t k = 0; k < 4; k++) {
                char_array_4[k] = base64_chars_index(char_array_4[k]);
            }
            
            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
            
            for (size_t k = 0; k < i - 1; k++) {
                output[j++] = char_array_3[k];
            }
        }
        
        return j;
    }

private:
    /**
     * @brief Check if a character is a valid Base64 character
     * 
     * @param c Character to check
     * @return true if valid Base64 character
     */
    static bool is_base64(uint8_t c) {
        return (isalnum(c) || (c == '+') || (c == '/'));
    }
    
    /**
     * @brief Get the index of a Base64 character
     * 
     * @param c Base64 character
     * @return Index in the Base64 character set
     */
    static uint8_t base64_chars_index(uint8_t c) {
        if (c >= 'A' && c <= 'Z') return c - 'A';
        if (c >= 'a' && c <= 'z') return c - 'a' + 26;
        if (c >= '0' && c <= '9') return c - '0' + 52;
        if (c == '+') return 62;
        if (c == '/') return 63;
        return 0;
    }
};

} // namespace util
} // namespace baja