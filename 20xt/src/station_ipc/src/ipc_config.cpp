#include <vector>
#include <string>
#include <mqueue.h>

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

// The data dispatching thread will need to access a queue for each reciever
const std::vector<int> StationIPC::get_rx_subsribers_qids()
{
  std::vector<int> qids = {
      open_queue(RX_QUEUE_SIMULATOR),
      open_queue(RX_QUEUE_LOGGER),
  };
  return qids;
}
