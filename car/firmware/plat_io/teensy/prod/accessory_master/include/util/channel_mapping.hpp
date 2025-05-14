/**
 * @file channel_mapping.hpp
 * @brief Utilities for mapping between ADC channel indices and semantic names (header-only)
 */

 #pragma once

 #include <string>
 #include <array>
 #include <unordered_map>
 
 namespace baja {
 namespace util {
 
 /**
  * @brief Enumeration of semantic channel names for ADC inputs
  */
 enum class ChannelName {
     GND,            // ADC AIN 0 - Ground reference
     FIVE_V_REFUL,   // ADC AIN 1 - 5V reference
     TWO_5_REF,      // ADC AIN 2 - 2.5V reference
     TWO_5_REFUL,    // ADC AIN 3 - 2.5V reference (buffered)
     SG2,            // ADC AIN 4 - Strain gauge 2
     SG1,            // ADC AIN 5 - Strain gauge 1
     CHANNEL_1,      // ADC AIN 6 - Channel 1
     CHANNEL_6,      // ADC AIN 7 - Channel 6
     CHANNEL_2,      // ADC AIN 8 - Channel 2
     CHANNEL_7,      // ADC AIN 9 - Channel 7
     CHANNEL_3,      // ADC AIN 10 - Channel 3
     CHANNEL_8,      // ADC AIN 11 - Channel 8
     CHANNEL_4,      // ADC AIN 12 - Channel 4
     CHANNEL_9,      // ADC AIN 13 - Channel 9
     CHANNEL_5,      // ADC AIN 14 - Channel 5
     CHANNEL_10,     // ADC AIN 15 - Channel 10
     UNKNOWN         // For invalid mappings
 };
 
 // Static array of all channel names in their index order
 static const std::array<ChannelName, 16> ALL_CHANNEL_NAMES = {
     ChannelName::GND,
     ChannelName::FIVE_V_REFUL,
     ChannelName::TWO_5_REF,
     ChannelName::TWO_5_REFUL,
     ChannelName::SG2,
     ChannelName::SG1,
     ChannelName::CHANNEL_1,
     ChannelName::CHANNEL_6,
     ChannelName::CHANNEL_2,
     ChannelName::CHANNEL_7,
     ChannelName::CHANNEL_3,
     ChannelName::CHANNEL_8,
     ChannelName::CHANNEL_4,
     ChannelName::CHANNEL_9,
     ChannelName::CHANNEL_5,
     ChannelName::CHANNEL_10
 };
 
 // String representations of channel names
 const std::unordered_map<ChannelName, std::string> CHANNEL_NAME_STRINGS = {
     {ChannelName::GND, "GND"},
     {ChannelName::FIVE_V_REFUL, "5V_RefUl"},
     {ChannelName::TWO_5_REF, "2.5V_Ref"},
     {ChannelName::TWO_5_REFUL, "2.5V_RefUl"},
     {ChannelName::SG2, "SG2"},
     {ChannelName::SG1, "SG1"},
     {ChannelName::CHANNEL_1, "Channel_1"},
     {ChannelName::CHANNEL_6, "Channel_6"},
     {ChannelName::CHANNEL_2, "Channel_2"},
     {ChannelName::CHANNEL_7, "Channel_7"},
     {ChannelName::CHANNEL_3, "Channel_3"},
     {ChannelName::CHANNEL_8, "Channel_8"},
     {ChannelName::CHANNEL_4, "Channel_4"},
     {ChannelName::CHANNEL_9, "Channel_9"},
     {ChannelName::CHANNEL_5, "Channel_5"},
     {ChannelName::CHANNEL_10, "Channel_10"},
     {ChannelName::UNKNOWN, "Unknown"}
 };
 
 /**
  * @brief Get the semantic name for an ADC channel index
  * @param channelIndex The ADC channel index (0-15)
  * @return The corresponding ChannelName enum value
  */
//  inline ChannelName getChannelNameFromIndex(uint8_t channelIndex) {
//      if (channelIndex < ALL_CHANNEL_NAMES.size()) {
//          return ALL_CHANNEL_NAMES[channelIndex];
//      }
//      return ChannelName::UNKNOWN;
//  }
 
 /**
  * @brief Get the ADC channel index for a channel name
  * @param channelName The ChannelName enum value
  * @return The corresponding ADC channel index (0-15)
  */
//  inline uint8_t getIndexFromChannelName(ChannelName channelName) {
//      for (uint8_t i = 0; i < ALL_CHANNEL_NAMES.size(); i++) {
//          if (ALL_CHANNEL_NAMES[i] == channelName) {
//              return i;
//          }
//      }
//      return UINT8_MAX; // Invalid index
//  }
 
 /**
  * @brief Get the string representation of a channel name
  * @param channelName The ChannelName enum value
  * @return String representation of the channel name
  */
 inline std::string getChannelNameString(ChannelName channelName) {
     auto it = CHANNEL_NAME_STRINGS.find(channelName);
     if (it != CHANNEL_NAME_STRINGS.end()) {
         return it->second;
     }
     return "Unknown";
 }
 
 /**
  * @brief Get the string representation of a channel name from an index
  * @param channelIndex The ADC channel index (0-15)
  * @return String representation of the channel name
  */
//  inline std::string getChannelNameString(uint8_t channelIndex) {
//      return getChannelNameString(getChannelNameFromIndex(channelIndex));
//  }
 
 /**
  * @brief Check if a channel should be enabled by default
  * 
  * This function determines if a channel should be enabled by default
  * based on its semantic role (e.g., reference channels, sensor channels)
  * 
  * @param channelName The ChannelName enum value
  * @return true if the channel should be enabled by default, false otherwise
  */
 inline bool shouldChannelBeEnabled(ChannelName channelName) {
     // Enable all data channels by default, but not reference channels
     switch (channelName) {
         case ChannelName::GND:
         case ChannelName::FIVE_V_REFUL:
         case ChannelName::TWO_5_REF:
         case ChannelName::TWO_5_REFUL:
         case ChannelName::SG1:
         case ChannelName::SG2:
             return false; // Reference channels disabled by default
         case ChannelName::CHANNEL_1:
         case ChannelName::CHANNEL_2:
         case ChannelName::CHANNEL_3:
         case ChannelName::CHANNEL_4:
         case ChannelName::CHANNEL_5:
         case ChannelName::CHANNEL_6:
         case ChannelName::CHANNEL_7:
         case ChannelName::CHANNEL_8:
         case ChannelName::CHANNEL_9:
         case ChannelName::CHANNEL_10:
             return true; // Data channels enabled by default
         default:
             return false;
     }
 }
 
 /**
  * @brief Get all available channel names
  * @return Array of all channel names
  */
 inline const std::array<ChannelName, 16>& getAllChannelNames() {
     return ALL_CHANNEL_NAMES;
 }
 
 } // namespace util
 } // namespace baja