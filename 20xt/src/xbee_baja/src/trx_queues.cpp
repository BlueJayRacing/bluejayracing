#include <map>
#include <iostream>

#include "ipc/safe_queue.h"
#include "interfaces/live_comm_queue.h"
#include "ipc/trx_queues.h"
#include "baja_live_comm.pb.h"

TRXProtoQueues::TRXProtoQueues(int max_queue_size)
{
  for (auto &queue : queues)
  {
    queue.second = new SafeLiveCommQueue(max_queue_size);
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
      total_size += queue.second->size();
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
  return queues.at(queue_id)->size();
}

bool TRXProtoQueues::enqueue(int queue_id, LiveComm data)
{
  const google::protobuf::Reflection *reflection = data.GetReflection();
  const google::protobuf::FieldDescriptor *field = data.GetDescriptor()->FindFieldByNumber(queue_id);
  if (field != nullptr)
  {
    if (!reflection->HasField(data, field))
    {
      std::cerr << "ERROR: Cannot enqueue. LiveComm object missing field #" << queue_id << std::endl;
      return false;
    }
  }

  bool success = queues.at(queue_id)->enqueue(data);
  return success;
}

LiveComm TRXProtoQueues::front(int queue_id)
{
  if (this->get_size(queue_id) <= 0)
  {
    std::cerr << "ERROR: No data in queue with field ID. Returning default LiveComm " << queue_id << std::endl;
    return LiveComm();
  }

  return queues.at(queue_id)->front();
}

LiveComm TRXProtoQueues::dequeue(int queue_id)
{
  if (this->get_size(queue_id) <= 0)
  {
    std::cerr << "ERROR: No data in queue with field ID. Returning default LiveComm " << queue_id << std::endl;
    return LiveComm();
  }

  return queues.at(queue_id)->dequeue();;
}