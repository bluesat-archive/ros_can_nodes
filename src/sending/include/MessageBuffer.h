#ifndef MESSAGE_BUFFER_H
#define MESSAGE_BUFFER_H

#include <queue>
#include <condition_variable>
#include <mutex>
#include <string>
#include <iostream>
#include <linux/can.h>

class MessageBuffer {
    private:
        std::queue<struct can_frame> q;
        std::condition_variable cv;
        std::mutex mutex;

        // synchronised
        struct can_frame pop();

    public:
        MessageBuffer() {}

        // synchronised
        void push(const struct can_frame& s);

        // starts a thread to loop sending messages from the buffer
        void startSendThread();
};

#endif // MESSAGE_BUFFER_H