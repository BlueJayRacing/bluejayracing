#include <unistd.h>
#include <iostream>
#include <vector>
#include <cassert>
#include "data_logging.h"

using std::cout;

int main()
{
  std::vector<char> cwd(1024);
  getcwd(cwd.data(), cwd.size());
  std::string cwd_str = cwd.data();

  std::string LOG_DIR = cwd_str + "/logs";
  std::string CHANNEL_NAME_1 = "adc_1";
  std::string CHANNEL_NAME_2 = "adc_2";

  std::cout << "Logging into directory: " << LOG_DIR << std::endl;

  /* Initialization Tests */
  BajaDataLogging::BajaDataWriter writer(LOG_DIR);
  writer.clear_config();
  try {
    writer.write_uint16(0 /* non-existant channel */, 255, 652);
    throw std::runtime_error("Test failed: can't write before adding channel");
  } catch (std::runtime_error) {
    cout << "test passed: cannot write before adding channel\n";
  }

  /* Config Editing Tests */
  uint8_t channel_id_1 = writer.add_channel(CHANNEL_NAME_1, "volts", "1.0", "miliseconds", "16");
  assert(channel_id_1 == 0); // otherwise tests will fail
  cout << "test passed: added a channel\n";

  try {
    writer.add_channel("invalid channel name", "volts", "1.0", "miliseconds", "16");
    throw std::runtime_error("Test failed: channel_name should not be able to contain spaces");
  } catch (std::runtime_error) {
    cout << "test passed: channel_name should not be able to contain spaces\n";
  }

  try {
    writer.add_channel(CHANNEL_NAME_1, "volts", "1.0", "miliseconds", "16");
    throw std::runtime_error("Test failed: cannot add channel with same name twice");
  } catch (std::runtime_error) {
    cout << "test passed: cannot add channel with the same name twice\n";
  }

  try {
    writer.add_channel("valid_channel_name_654685118", "volts", "1.0", "miliseconds", "64");
    throw std::runtime_error("Test failed: cannot have more than 32 bits of data per sample");
  } catch (std::runtime_error) {
    cout << "test passed: cannot have more than 32 bits of data per sample\n";
  }

  /* Data writing tests w/ example outputs*/
  try {
    writer.write_uint16(165 /* non-existant channel */, 255, 652);
    throw std::runtime_error("Test failed: cannot write to a channel which does not exist");
  } catch (std::runtime_error) {
    cout << "test passed: cannot write to a channel which does not exist\n";
  }

  writer.write_uint16(channel_id_1 /*00*/, 65535 /*ff ff*/, 0 /*00 00 00 00 00 00 00 00*/); 
  cout << "test passed: wrote a 1st uint16\n";
  writer.write_uint16(channel_id_1 /*00*/, 61455 /*0f f0*/, 0 /*00 00 00 00 00 00 00 00*/);
  cout << "test passed: wrote a 2nd uint16\n";
  writer.write_uint16(channel_id_1 /*00*/, 61455 /*ff ff*/, 18374686479671623680 /*00 00 00 00 00 00 00 ff*/);
  cout << "test passed: wrote a 3rd uint16\n";

  uint8_t channel_id_2 = writer.add_channel(CHANNEL_NAME_2, "volts", "1.0", "miliseconds", "16");
  assert(channel_id_2 == (uint8_t) 1); // otherwise tests will break
  writer.write_uint16(channel_id_2 /*01*/, 65535 /*ff ff*/, 0 /*00 00 00 00 00 00 00 00*/);
  cout << "test passed: wrote a 4th uint16\n";

  /* Opens second file when needed */
  const int num_bytes_per_sample = (8 + 16 + 16 + 64) / 8;
  const int num_samples_needed = BajaDataLogging::MAX_DATA_FILE_SIZE / num_bytes_per_sample;
  for (int i=0; i <= num_samples_needed; i++) {
    writer.write_uint16(channel_id_1, 65535, 0);
  }
  cout << "test ? passed: check if there is a second file\n";

  return 0;
}