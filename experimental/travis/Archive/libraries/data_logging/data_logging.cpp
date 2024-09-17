#include <fstream>
#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include "data_logging.h"
#include "_channel_description.h"

using namespace BajaDataLogging;
using std::ofstream;
using std::ifstream;
using std::string;


BajaDataWriter::BajaDataWriter(const string log_directory) : 
  log_directory(log_directory),
  config_file_path(log_directory + "/" + CONFIG_FILE_NAME),
  current_data_file_num(-1),
  current_data_file_size(0)
{
  // Check if log directory exists, if not create it
  if (!std::filesystem::exists(log_directory)) {
    std::filesystem::create_directory(log_directory);
  }
  this->read_or_create_config();
  this->open_new_data_file();
}

int BajaDataWriter::add_channel(
    string channel_name,
    string unit_of_measurement,
    string scale_factor_decimal, // string representation, eg "2350.024"
    string time_unit,
    string num_bits_per_sample)
{
  std::string all_strings = channel_name + unit_of_measurement + scale_factor_decimal + time_unit + num_bits_per_sample;
  if (all_strings.find(',') != std::string::npos 
      || all_strings.find(':') != std::string::npos 
      || all_strings.find('"') != std::string::npos
      || all_strings.find(' ') != std::string::npos
      || all_strings.find('\n') != std::string::npos
      || all_strings.find('{') != std::string::npos
      || all_strings.find('}') != std::string::npos
      ) {
    throw std::runtime_error("Cannot use a specific set characters in any of the strings");
  }

  // channel does not already exist
  for (const auto& channel_desc : this->channels) {
    if (channel_desc.channel_name == channel_name) {
      throw std::runtime_error("Channel already exists");
    }
  }

  // num_bits_per_sample is <= DATA_BITS_PER_SAMPLE
  if (std::stoi(num_bits_per_sample) > DATA_BITS_PER_SAMPLE) {
    throw std::runtime_error("num_bits_per_sample is limited, see code for the constant");
  }

  // Store the channel and update the config
  uint8_t channel_id = this->channels.size();
  ChannelDescription channel_description(channel_name, unit_of_measurement, scale_factor_decimal, time_unit, num_bits_per_sample, channel_id);
  this->channels.push_back(channel_description);
  this->overwrite_config();
  return channel_id;
}

void BajaDataWriter::clear_config() {
  std::cout << "clearing the config as instructed" << std::endl;
  this->channels.clear();
  this->overwrite_config();
}

int BajaDataWriter::write_uint16(uint8_t channel_id, uint16_t data, uint64_t timestamp)
{
  // Check if channel exists
  if (channel_id >= channels.size()) {
    throw std::runtime_error("Channel does not exist");
  }

  uint16_t padding = 0;
  int entry_size = sizeof(channel_id) + sizeof(padding) + sizeof(data) + sizeof(timestamp);

  // write the binary
  this->current_data_file.write(reinterpret_cast<const char*>(&channel_id), sizeof(channel_id)); // 2 hex
  this->current_data_file.write(reinterpret_cast<const char*>(&padding), sizeof(padding)); // 4 hex
  this->current_data_file.write(reinterpret_cast<const char*>(&data), sizeof(data)); // 4 hex
  this->current_data_file.write(reinterpret_cast<const char*>(&timestamp), sizeof(timestamp)); // 16 hex
  this->current_data_file_size += entry_size;

  // If another entry would overflow the file, open a new one
  if (current_data_file_size > MAX_DATA_FILE_SIZE - entry_size) {
    this->open_new_data_file();
  }
  return 0;
}

int BajaDataWriter::read_or_create_config()
{
  // If config file DNE, create an empty file
  if (!std::filesystem::exists(this->config_file_path)) {
    std::cout << "config file does not exist, creating it\n";
    std::ofstream empty_file(this->config_file_path);
    if (!empty_file.is_open()) {
      throw std::runtime_error("Could not create empty file");
    }
    empty_file.close();
  }

  ifstream config_file(this->config_file_path);
  if (!config_file.is_open()) {
    throw std::runtime_error("Could not open config file");
  }

  // Split the file by new-lines and add it to jsons
  std::vector<std::string> jsons;
  std::string line;
  while (std::getline(config_file, line)) {
    jsons.push_back(line);
  }

  // for each json in lines, create a ChannelDescription and add to channel_map
  for (std::string json : jsons) {
    try {
      ChannelDescription channel_description(json);
      channels.push_back(channel_description);
    } catch (const std::invalid_argument& e) {
      std::cerr << "Invalid json in config file, completely overwriting with empty config. Reason: " << e.what() << std::endl; 
      std::cerr << "Invalid json: " << json << std::endl;
      this->channels.clear();
      this->overwrite_config();
      continue;
    }
  }
  config_file.close();
  return 0;
}

int BajaDataWriter::overwrite_config()
{
  ofstream config_file(this->config_file_path);
  if (!config_file.is_open()) {
    throw std::runtime_error("Could not open config file");
  }

  // New-line delineated JSONs
  for (auto& channel_description : this->channels) {
    config_file << channel_description.to_json() << std::endl;
  }
  config_file.close();
  return 0;
}

int BajaDataWriter::open_new_data_file()
{
  this->current_data_file.close();
  this->current_data_file_size = 0;
  this->current_data_file_num++;
  string data_file_path = this->log_directory + "/" + DATA_FILE_PREFIX + std::to_string(this->current_data_file_num) + DATA_FILE_EXTENSION;
  this->current_data_file.open(data_file_path);
  return 0;
}
