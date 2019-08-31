#ifndef CAN_SEND_QUEUE_HPP
#define CAN_SEND_QUEUE_HPP

#include <queue>
#include <condition_variable>
#include <mutex>
#include <linux/can.h>

class CANSendQueue {
    public:
        static CANSendQueue& instance();

        // thread-safe frame push
        void push(const can_frame& frame);

        CANSendQueue(const CANSendQueue&) = delete;
        void operator=(const CANSendQueue&) = delete;
    
    private:
        // starts the sender thread
        CANSendQueue();

        std::queue<can_frame> q;
        std::condition_variable cv;
        std::mutex mutex;
};

#endif // CAN_SEND_QUEUE_HPP
