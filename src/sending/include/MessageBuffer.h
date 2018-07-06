#ifndef MESSAGE_BUFFER_H
#define MESSAGE_BUFFER_H

#include <queue>
#include <condition_variable>
#include <mutex>
#include <string>
#include <iostream>

class MessageBuffer {
    private:
        std::queue<std::string> q;
        std::condition_variable cv;
        std::mutex mutex;

        // synchronised
        std::string pop();

    public:
        MessageBuffer() {}

        // synchronised
        void push(const std::string& s);

        // starts a thread to loop sending messages from the buffer
        void startSendThread();
};

#endif // MESSAGE_BUFFER_H