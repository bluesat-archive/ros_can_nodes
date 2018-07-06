#include "MessageBuffer.h"
#include <iostream>
#include <string>
#include <mutex>
#include <condition_variable>
#include <thread>

void MessageBuffer::push(const struct can_frame& data) {
    std::unique_lock<std::mutex> lk(mutex);

    q.push(data);
    
    lk.unlock();
    cv.notify_all();
}

struct can_frame MessageBuffer::pop() {
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
            //std::cout << "sending " << data << std::endl;
            std::cout << data.can_id << ":";
            for (int i = 0;i < data.can_dlc;++i) {
                std::cout << data.data[i];
            }
            std::cout << "\n";

            // TODO send code here

            // debug exit condition
            if (data.__res0 == 8) {
                std::cout << "exit condition\n";
                break;
            }
        }
    }};
    sender.detach();
}