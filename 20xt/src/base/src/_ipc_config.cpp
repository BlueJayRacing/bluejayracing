#include <vector>
#include <string>
#include <mqueue.h>
#include <stdexcept>
#include <iostream>

#include "helpers/ipc_config.h"

const mqd_t StationIPC::open_queue(std::string q_fname)
{
  return mq_open(q_fname.c_str(), QUEUE_FLAGS, QUEUE_MODE, &QUEUE_ATTRIBUTES);
}

const int StationIPC::close_queue(mqd_t qid)
{
  mq_close(qid);
  return EXIT_SUCCESS;
}

const int StationIPC::unlink_queue(std::string q_fname)
{
  mq_unlink(q_fname.c_str());
  return EXIT_SUCCESS;
}

const std::string StationIPC::get_message(mqd_t qid) {
  char buffer[StationIPC::MAX_QUEUE_MSG_SIZE];
  int bytes_read = mq_receive(qid, buffer, StationIPC::MAX_QUEUE_MSG_SIZE, NULL);
  if (bytes_read == -1) {
    if (errno == EAGAIN) {
      return ""; // empty queue
    }
    std::cerr << "ERROR: Could not read the message. Errno " << errno << std::endl;
    return "";
  }
  return std::string(buffer, bytes_read);
}

const int StationIPC::send_message(mqd_t qid, const std::string& payload) {
  if (payload.size() > StationIPC::MAX_QUEUE_MSG_SIZE) {
    std::cerr << "ERROR: Message is too long" << std::endl;
    return SEND_ERROR;
  }

  int err = mq_send(qid, payload.c_str(), payload.size(), 0);
  while (err == -1 && errno == EAGAIN) {
    return QUEUE_FULL;
  }
  if (err == -1) {
    std::cerr << "ERROR: Could not send the message. Errno " << errno << std::endl;
    return SEND_ERROR;
  }
  return SUCCESS;
}

