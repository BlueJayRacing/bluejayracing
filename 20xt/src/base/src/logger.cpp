#include <iostream>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include "ipc_config.h"

// An infinite loop which attempts to read bytes from an ipc queue
int main() {
  int qid = StationIPC::get_qid(StationIPC::RX_QID_FOR_LOGGER_FNAME);
  if (qid == -1) {
    std::cout << "Failed to get queue ID" << std::endl;
    return EXIT_FAILURE;
  }

  int msgtype = 1;
  int msgflag = 0 | IPC_NOWAIT;

  char data = '\0';
  int err = msgrcv(qid, &data, sizeof(data), msgtype, msgflag);
  if (err == -1) {
    std::cout << "Could not read the message" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Read data: '" << data << "'" << std::endl; 
  return EXIT_SUCCESS;
}