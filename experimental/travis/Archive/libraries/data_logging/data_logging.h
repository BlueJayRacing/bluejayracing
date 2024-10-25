#ifndef BAJA_DATA_LOGGING_H
#define BAJA_DATA_LOGGING_H

#include "_channel_description.h"
#include <fstream>
#include <string>
#include <vector>
using std::ofstream;
using std::string;
namespace BajaDataLogging
{

const string CONFIG_FILE_NAME    = "meta_data_config.json";
const string DATA_FILE_EXTENSION = ".bin";
const string DATA_FILE_PREFIX    = "data_";
const int MAX_DATA_FILE_SIZE     = 100000000; // 100 MB
const int DATA_BITS_PER_SAMPLE   = 32;        // Must be 16, 32, 64, etc

class BajaDataWriter {
  public:
    /* A class for logging channel data. Stores everything in LITTLE-ENDIAN*/
    BajaDataWriter(string log_directory);

    /* Return an integer which will identify this channel */
    int add_channel(string channel_name, string unit_of_measurement,
                    string scale_factor_decimal, // string representation, eg "2350.024"
                    string time_unit, string num_bits_per_sample);

    int write_uint16(uint8_t channel_id, uint16_t data, uint64_t timestamp);
    void clear_config();

  private:
    const string log_directory;
    const string config_file_path;
    std::vector<ChannelDescription> channels;
    ofstream current_data_file;
    int current_data_file_num;
    int current_data_file_size; // bytes

    int read_or_create_config();
    int overwrite_config();
    int open_new_data_file();
};

class BajaDataReader {};

} // namespace BajaDataLogging

#endif