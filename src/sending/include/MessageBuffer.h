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

        // precondition: q.size() > 0
        std::string send_next();

    public:
        MessageBuffer() {}

        // synchronised
        void push(const std::string& s);

        // synchronised
        std::string send(const bool flush = false);
};

#endif // MESSAGE_BUFFER_H