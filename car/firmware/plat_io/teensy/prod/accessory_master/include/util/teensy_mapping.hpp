/**
 * @file teensy_mapping.hpp
 * @brief Defines the internal channel ID system for the Teensy data acquisition system
 */

 #pragma once

 #include <cstdint>
 #include <string>
 #include <array>
 #include <unordered_map>
 
 namespace baja {
 namespace util {
 
 /**
  * @brief Internal channel ID enumeration
  * 
  * Maps all possible data sources in the system to a unified ID space
  * - Analog channels (ADC AIN 0-15): IDs 0-15
  * - Digital channels (DIN 0-5): IDs 16-21 
  * - MISC channels: IDs 22-29
  */
 enum class InternalChannelID : uint8_t {
     // Analog channels (ADC)
     AIN0 = 0,   // Ground reference
     AIN1,       // 5V reference
     AIN2,       // 2.5V reference
     AIN3,       // 2.5V reference (buffered)
     AIN4,       // Strain gauge 2
     AIN5,       // Strain gauge 1
     AIN6,       // Channel 1
     AIN7,       // Channel 6
     AIN8,       // Channel 2 
     AIN9,       // Channel 7
     AIN10,      // Channel 3
     AIN11,      // Channel 8
     AIN12,      // Channel 4
     AIN13,      // Channel 9
     AIN14,      // Channel 5
     AIN15,      // Channel 10
     
     // Digital channels
     DIN0 = 16,
     DIN1,
     DIN2,
     DIN3,
     DIN4,
     DIN5,
     
     // Miscellaneous channels
     MISC0 = 22,  // Can be used for system temperature
     MISC1,       // Can be used for power supply voltage
     MISC2,       // Can be used for CPU load
     MISC3,       // Can be used for memory usage
     MISC4,
     MISC5,
     MISC6,
     MISC7,
     
     UNKNOWN = 127  // For invalid mappings, keep under 128 for varint sizing
 };
 
 // Total number of channels in the system
 constexpr uint8_t TOTAL_CHANNEL_COUNT = 30;
 
 // String representations of channel IDs with descriptive names
 const std::array<std::string, TOTAL_CHANNEL_COUNT> CHANNEL_NAMES = {
     "ADC AIN 0 - Ground reference",
     "ADC AIN 1 - 5V reference",
     "ADC AIN 2 - 2.5V reference",
     "ADC AIN 3 - 2.5V reference (buffered)",
     "ADC AIN 4 - Strain gauge 2",
     "ADC AIN 5 - Strain gauge 1",
     "ADC AIN 6 - Channel 1", 
     "ADC AIN 7 - Channel 6",
     "ADC AIN 8 - Channel 2",
     "ADC AIN 9 - Channel 7",
     "ADC AIN 10 - Channel 3",
     "ADC AIN 11 - Channel 8",
     "ADC AIN 12 - Channel 4",
     "ADC AIN 13 - Channel 9",
     "ADC AIN 14 - Channel 5",
     "ADC AIN 15 - Channel 10",
     "DIN 0",
     "DIN 1",
     "DIN 2",
     "DIN 3",
     "DIN 4",
     "DIN 5",
     "MISC 0 - System temperature",
     "MISC 1 - Power supply",
     "MISC 2 - CPU load",
     "MISC 3 - Memory usage",
     "MISC 4",
     "MISC 5",
     "MISC 6", 
     "MISC 7"
 };
 
 /**
  * @brief Map ADC channel (AIN) index to internal channel ID
  * 
  * @param adcChannel The ADC channel index (0-15)
  * @return The corresponding internal channel ID
  */
 inline InternalChannelID mapADCToInternalID(uint8_t adcChannel) {
     if (adcChannel <= 15) {
         return static_cast<InternalChannelID>(adcChannel);
     }
     return InternalChannelID::UNKNOWN;
 }
 
 /**
  * @brief Get the string representation of a channel ID
  * 
  * @param channelID The internal channel ID
  * @return String representation with descriptive name
  */
 inline std::string getChannelName(InternalChannelID channelID) {
     uint8_t index = static_cast<uint8_t>(channelID);
     if (index < TOTAL_CHANNEL_COUNT) {
         return CHANNEL_NAMES[index];
     }
     return "Unknown Channel";
 }
 
 /**
  * @brief Get the string representation of a channel ID from raw uint8_t
  * 
  * @param channelID The internal channel ID as uint8_t
  * @return String representation with descriptive name
  */
 inline std::string getChannelName(uint8_t channelID) {
     if (channelID < TOTAL_CHANNEL_COUNT) {
         return CHANNEL_NAMES[channelID];
     }
     return "Unknown Channel";
 }
 
 /**
  * @brief Check if a channel should be enabled by default
  * 
  * @param channelID The internal channel ID
  * @return true if the channel should be enabled by default
  */
 inline bool shouldChannelBeEnabled(InternalChannelID channelID) {
     uint8_t id = static_cast<uint8_t>(channelID);
     
     // Specialized enabling logic for different channel types
     if (id <= 15) {
         // ADC channels - enable all data channels but not reference channels
         switch (id) {
             case 0: // Ground reference
             case 1: // 5V reference
             case 2: // 2.5V reference
             case 3: // 2.5V reference (buffered)
                 return false;
             case 4: // Strain gauge 2
             case 5: // Strain gauge 1
                 return true; // Enable strain gauges by default
             default:
                 return true; // Enable all other ADC channels
         }
     } 
     else if (id >= 16 && id <= 21) {
         // Digital channels - enable all by default
         return true;
     }
     else if (id >= 22 && id <= 29) {
         // Misc channels - disable by default, enable programmatically as needed
         return false;
     }
     
     return false;
 }
 
 /**
  * @brief Generate a CSV mapping header with all channel mappings
  * 
  * Creates a commented line for the CSV file that maps all channel IDs to names
  * 
  * @return String containing the CSV mapping header
  */
 inline std::string generateChannelMappingHeader() {
     std::string header = "# Channel ID mappings: ";
     
     for (uint8_t i = 0; i < TOTAL_CHANNEL_COUNT; i++) {
         header += std::to_string(i) + ":\"" + CHANNEL_NAMES[i] + "\"";
         if (i < TOTAL_CHANNEL_COUNT - 1) {
             header += ", ";
         }
     }
     
     return header + "\r\n";
 }
 
 } // namespace util
 } // namespace baja