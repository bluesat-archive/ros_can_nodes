#ifndef MESSAGE_BUFFER_HPP
#define MESSAGE_BUFFER_HPP

#include <vector>
#include <queue>
#include <condition_variable>
#include <mutex>
#include <linux/can.h>

class MessageBuffer {
    public:
        static MessageBuffer& instance();

        // thread-safe frame push
        void push(const can_frame& frame);

        MessageBuffer(const MessageBuffer&) = delete;
        void operator=(const MessageBuffer&) = delete;
    
    private:
        // starts the sender thread
        MessageBuffer();

        std::queue<can_frame> q;
        std::condition_variable cv;
        std::mutex mutex;
};

#endif // MESSAGE_BUFFER_HPP