#include <unistd.h>
#include <iostream>
#include <vector>

#include "data_logging.h"

using std::cout;

int main()
{
  std::vector<char> cwd(1024);
  getcwd(cwd.data(), cwd.size());
  std::string cwd_str = cwd.data();

  std::string LOG_DIR = cwd_str + "/logs";
  std::string CHANNEL_NAME_1 = "adc_1";

  std::cout << "Logging into directory: " << LOG_DIR << std::endl;

  /* Initialization Tests */
  BajaDataLogging::BajaDataWriter writer(LOG_DIR);
  try {
    writer.write_uint16("non-existant-chanenl", 255, 652);
    throw std::runtime_error("Test failed: can't write before adding channel");
  } catch (std::runtime_error) {
    cout << "test passed: cannot write before adding channel\n";
  }

  /* Config Editing Tests */
  writer.add_channel(CHANNEL_NAME_1, "volts", "1.0", "miliseconds", "16");
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


  /* Data writing tests */
  try {
    writer.write_uint16("non-existant-chanenl", 255, 652);
    throw std::runtime_error("Test failed: cannot write to a channel which does not exist");
  } catch (std::runtime_error) {
    cout << "test passed: cannot write to a channel which does not exist\n";
  }

  writer.write_uint16(CHANNEL_NAME_1, 65535, 0);
  cout << "test passed: wrote a uint16\n";
  
  return 0;
}