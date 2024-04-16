#include <iostream>
#include <mqueue.h>
#include <unistd.h>

#include "mains/data_logger.h"
#include "ipc_config.h"
#include "baja_live_comm.pb.h"


#include <chrono>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <google/protobuf/util/json_util.h>


/* Return the base of the filename which things get logged to "log_dtype" */
std::string get_base_log_filename(Observation &observation)
{
  std::string filename;
  switch (observation.data_case())
  {
  case Observation::kGps:
  {
    filename = "log_gps";
  }
  break;
  case Observation::kLocalization:
  {
    filename = "log_localization";
  }
  break;
  case Observation::kCommunication:
  {
    filename = "log_communication";
  }
  break;
  case Observation::kAnalogCh:
  {
    filename = "log_analog_ch";
  }
  break;
  case Observation::kCarState:
  {
    filename = "log_car_state";
  }
  break;
  case Observation::kRtkCorrection:
  {
    filename = "log_rtk_correction";
  }
  break;
  case Observation::DATA_NOT_SET:
    break;
  }
  return filename;
}


int main()
{
  std::cout << "starting data logger..." << std::endl;

  // Open queue
  const mqd_t rx_queue = BajaIPC::open_queue(StationIPC::RECEIVE_DISPATCHER_TO_DATA_LOGGER, true);
  if (rx_queue == -1)
  {
    std::cout << "Failed to get recieve queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  // Main loop
  auto log_time = std::chrono::system_clock::now();
  while (true)
  {
    std::string msg = BajaIPC::get_message(rx_queue); // Blocking
    if (msg == "")
    {
      continue;
    }
    std::cout << "got a msg" << std::endl;

    Observation observation;
    observation.ParseFromString(msg);
    std::string filename = get_base_log_filename(observation);
    if (filename.empty()) {
      continue;
    }

    std::cout << "got a fname: " << filename << std::endl;

    // Update filename if 5 minutes have passed
    auto currentTime = std::chrono::system_clock::now();
    auto timeDiff = std::chrono::duration_cast<std::chrono::minutes>(currentTime - log_time);
    if (timeDiff.count() >= 5)
    {
      // Update last log time
      log_time = currentTime;
    }

    // Format time as HH:MM:SS
    std::time_t log_time_t = std::chrono::system_clock::to_time_t(log_time);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&log_time_t), "%T");
    std::string formattedTime = ss.str();
    filename += "_" + formattedTime;

    std::ofstream log_file(filename, std::ios::app);
    if (log_file.is_open())
    {
      std::string json_string;
      google::protobuf::util::MessageToJsonString(observation, &json_string);
      log_file << json_string << std::endl;
      log_file.close();
    }
  }
  return EXIT_SUCCESS;
}
