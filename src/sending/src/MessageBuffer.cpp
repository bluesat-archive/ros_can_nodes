#include "MessageBuffer.h"
#include <iostream>
#include <string>
#include <mutex>
#include <condition_variable>

std::string MessageBuffer::send_next() {
    auto data = q.front();
    q.pop();
    std::cout << "sending " << data << "\n";
    return data;
}

void MessageBuffer::push(const std::string& s) {
    std::unique_lock<std::mutex> lk(mutex);
    
    q.push(s);
    
    lk.unlock();
    cv.notify_all();
}

std::string MessageBuffer::send(const bool flush) {
    std::unique_lock<std::mutex> lk(mutex);
    cv.wait(lk, [this](){ return q.size() > 0; });
    
    auto data = send_next();
    if (flush) {
        while (q.size() > 0) {
            data = send_next();
        }
    }
    lk.unlock();
    return data;
}