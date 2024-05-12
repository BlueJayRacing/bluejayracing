#ifndef BAJA_DATA_LOGGING_H
#define BAJA_DATA_LOGGING_H

#include <fstream>
#include <string>
#include <map>
#include "_channel_description.h"

using std::string;
using std::ofstream;

namespace BajaDataLogging {

typedef std::map<string, ChannelDescription> ChannelMap;
const string CONFIG_FILE_NAME = "meta_data_config.json";
const string DATA_FILE_EXTENSION = ".bin";
const string DATA_FILE_PREFIX = "data_";
const int APPROX_MAX_DATA_FILE_SIZE = 100000000; // 100 mb
const int DATA_BITS_PER_SAMPLE = 32; // Must be 16, 32, 64, etc


class BajaDataWriter
{
public:
  BajaDataWriter(string log_directory);

  int add_channel(
      string channel_name,
      string unit_of_measurement,
      string scale_factor_decimal, // string representation, eg "2350.024"
      string time_unit,
      string num_bits_per_sample);

  int write_uint16(string channel_name, uint16_t data, uint64_t timestamp);

private:
  string log_directory;
  ChannelMap channel_map; // If too slow can replace with vector I guess
  ofstream current_data_file;
  int current_data_file_num;
  int current_data_file_size; // bytes
  
  int read_or_create_config();
  int overwrite_config();
  int open_new_data_file();
};

class BajaDataReader
{
};

} // endnamespace


#endif