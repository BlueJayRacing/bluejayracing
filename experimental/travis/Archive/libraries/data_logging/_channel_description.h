#ifndef _CHANNEL_DESCRIPTION_H
#define _CHANNEL_DESCRIPTION_H

#include <string>

class ChannelDescription
{
public:
  ChannelDescription(
      std::string channel_name,
      std::string unit_of_measurement,
      std::string scale_factor_decimal,
      std::string time_unit,
      std::string num_bits_per_sample,
      uint8_t channel_id)
      : channel_name(channel_name),
        unit_of_measurement(unit_of_measurement),
        scale_factor_decimal(scale_factor_decimal),
        time_unit(time_unit),
        num_bits_per_sample(num_bits_per_sample),
        channel_id(channel_id)
  {
  };

  ChannelDescription(std::string source_json);
  std::string to_json();
  
  std::string channel_name;
  std::string unit_of_measurement;
  std::string scale_factor_decimal;
  std::string time_unit;
  std::string num_bits_per_sample;
  uint8_t channel_id;

private:
  static const int NUM_ATTRIBUTES = 6; // Don't forget to change if needed!!
  int assign_value(std::string attribute_name, std::string value);
};

#endif