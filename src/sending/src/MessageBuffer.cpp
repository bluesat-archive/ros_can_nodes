#include "MessageBuffer.h"
#include <iostream>
#include <string>
#include <mutex>
#include <condition_variable>
#include <thread>

void MessageBuffer::push(const std::string& s) {
    std::unique_lock<std::mutex> lk(mutex);
    
    q.push(s);
    
    lk.unlock();
    cv.notify_all();
}

std::string MessageBuffer::pop() {
    std::unique_lock<std::mutex> lk(mutex);
    cv.wait(lk, [this](){ return q.size() > 0; });

    auto data = q.front();
    q.pop();
    
    lk.unlock();
    return data;
}

void MessageBuffer::startSendThread() {
    std::thread sender{[this]() {
        while (true) {
            auto data = pop();
            std::cout << "sending " << data << std::endl;

            // TODO send code here

            // debug exit condition
            if (data == "") {
                std::cout << "exit condition\n";
                break;
            }
        }
    }};
    sender.detach();
}