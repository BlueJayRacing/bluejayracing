#include <iostream>
#include <mqueue.h>
#include <stdexcept>
#include <string>
#include <vector>

#include "ipc_config.h"

/* Open a POSIX message queue. You can choose to open as blocking or not */
const mqd_t BajaIPC::open_queue(std::string q_fname, bool blocking)
{
    if (blocking) {
        return mq_open(q_fname.c_str(), QUEUE_FLAGS, QUEUE_MODE, &QUEUE_ATTRIBUTES);
    } else {
        return mq_open(q_fname.c_str(), QUEUE_FLAGS | O_NONBLOCK, QUEUE_MODE, &QUEUE_ATTRIBUTES);
    }
}

/* Get number of messages available to be dequeued */
const int num_messages_available(mqd_t qid)
{
    struct mq_attr attr;
    if (mq_getattr(qid, &attr) == -1) {
        perror("mq_getattr failed:");
    }
    return attr.mq_curmsgs;
}

/* Close the queue from your process' perspective. Does not destroy queue. */
const int BajaIPC::close_queue(mqd_t qid)
{
    mq_close(qid);
    return 0;
}

/* Destroys the queue. Likely unnecessary, will break your processes if called concurently */
const int BajaIPC::unlink_queue(std::string q_fname)
{
    mq_unlink(q_fname.c_str());
    return 0;
}

/* Attempt to enqueue a message. If queue is non-blocking, will return empty str if queue empty */
const std::string BajaIPC::get_message(mqd_t qid)
{
    char buffer[BajaIPC::MAX_QUEUE_MSG_SIZE];
    int bytes_read = mq_receive(qid, buffer, BajaIPC::MAX_QUEUE_MSG_SIZE, NULL);
    if (bytes_read == -1) {
        if (errno == EAGAIN) {
            return ""; // empty queue
        }
        std::cerr << "ERROR: Could not read the message. Errno " << errno << std::endl;
        return "";
    }
    return std::string(buffer, bytes_read);
}

/* Attempt to enqueue a message. If queue is non-blocking, may return QUEUE_FULL */
const int BajaIPC::send_message(mqd_t qid, const std::string& payload)
{
    if (payload.size() > BajaIPC::MAX_QUEUE_MSG_SIZE) {
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
