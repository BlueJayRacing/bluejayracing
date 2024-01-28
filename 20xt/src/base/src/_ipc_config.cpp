#include <vector>
#include <string>
#include <mqueue.h>
#include <stdexcept>

#include "ipc_config.h"

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

const std::string get_message(mqd_t qid) {
  throw std::runtime_error("Not implemented");
}

const int send_message(mqd_t qid, std::string msg) {
  throw std::runtime_error("Not implemented");

      std::string payload = observations[i].SerializeAsString();
    if (payload.size() > StationIPC::MAX_MSG_SIZE) {
      std::cout << "Message is too long" << std::endl;
    }

    int err = mq_send(qid, payload.c_str(), payload.size(), 0);
    while (err == -1 && errno == EAGAIN) {
      std::cout << "Queue is full, waiting for empty queue" << std::endl;
      std::this_thread::sleep_for(std::chrono::microseconds(1));
      err = mq_send(qid, payload.c_str(), payload.size(), 0);
    }

    if (err == -1) {
      std::cout << "Could not send the message. Errno " << errno << std::endl;
    }
}

