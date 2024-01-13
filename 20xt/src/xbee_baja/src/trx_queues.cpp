#include <map>
#include <iostream>

#include "ipc/safe_queue.h"
#include "ipc/trx_queues.h"
#include "baja_live_comm.pb.h"

TRXProtoQueues::TRXProtoQueues(int max_queue_size)
{
  for (auto &queue : queues)
  {
    queue.second = new SafeQueue<LiveComm>(max_queue_size);
  }
}

TRXProtoQueues::~TRXProtoQueues()
{
  for (auto &queue : queues)
  {
    if (queue.second != nullptr)
    {
      delete queue.second;
    }
  }
}

int TRXProtoQueues::get_total_size()
{
  int total_size = 0;
  for (auto &queue : queues)
  {
    if (queue.second != nullptr)
    {
      total_size += queue.second->get_size();
    }
  }
  return total_size;
}

int TRXProtoQueues::get_size(int queue_id)
{
  if (queues.find(queue_id) == queues.end())
  {
    std::cerr << "WARNING: No queue with field ID, returning size of 0 " << queue_id << std::endl;
    return 0;
  }
  if (queues.at(queue_id) == nullptr)
  {
    std::cerr << "WARNING: Queue with field ID is null, returning size of 0 " << queue_id << std::endl;
    return 0;
  }
  return queues.at(queue_id)->get_size();
}

void TRXProtoQueues::enqueue(int queue_id, LiveComm data)
{

  // TODO: Check to make sure the LiveComm has the field set (possible solution below)
  // const google::protobuf::Reflection *reflection = message.GetReflection();
  // const google::protobuf::FieldDescriptor *field = message.GetDescriptor()->FindFieldByNumber(queue_id);
  // if (field != nullptr)
  // {
  //   if (!reflection->HasField(data, field))
  //   {
  //     std::cout << "ERROR: LiveComm object missing field #" << queue_id << std::endl;
  //     return;
  //   }
  // }
  queues.at(queue_id)->enqueue(data);
}

LiveComm TRXProtoQueues::peek(int queue_id)
{
  if (queues.find(queue_id) == queues.end())
  {
    std::cerr << "ERROR: No queue with field ID. Returning default LiveComm " << queue_id << std::endl;
    return LiveComm();
  }

  if (queues.at(queue_id)->get_size() <= 0)
  {
    std::cerr << "ERROR: No data in queue with field ID. Returning default LiveComm " << queue_id << std::endl;
    return LiveComm();
  }

  return queues.at(queue_id)->peek();
}

LiveComm TRXProtoQueues::dequeue(int queue_id)
{
  if (queues.find(queue_id) == queues.end())
  {
    std::cerr << "ERROR: No queue with field ID. Returning default LiveComm " << queue_id << std::endl;
    return LiveComm();
  }

  if (queues.at(queue_id)->get_size() <= 0)
  {
    std::cerr << "ERROR: No data in queue with field ID. Returning default LiveComm " << queue_id << std::endl;
    return LiveComm();
  }

  return queues.at(queue_id)->dequeue();
}