#include <unistd.h>
#include <iostream>
#include <vector>

#include "data_logging.h"

int main()
{
  std::vector<char> cwd(1024);
  getcwd(cwd.data(), cwd.size());
  std::string cwd_str = cwd.data();
  std::string logs_dir = cwd_str + "/logs";

  std::cout << "Logging into directory: " << logs_dir << std::endl;

  BajaDataLogging::BajaDataWriter writer(logs_dir);
  return 0;
}