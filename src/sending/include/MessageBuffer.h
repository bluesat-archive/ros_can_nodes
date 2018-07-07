#ifndef MESSAGE_BUFFER_H
#define MESSAGE_BUFFER_H

#include <queue>
#include <condition_variable>
#include <mutex>
#include <linux/can.h>

class MessageBuffer {
    public:
        static MessageBuffer& instance() noexcept;

        // thread-synchronised
        void push(const struct can_frame& s);

        MessageBuffer(const MessageBuffer&) = delete;
        void operator=(const MessageBuffer&) = delete;
    
    private:
        // starts the sender thread
        MessageBuffer();

        std::queue<struct can_frame> q;
        std::condition_variable cv;
        std::mutex mutex;
};

#endif // MESSAGE_BUFFER_H