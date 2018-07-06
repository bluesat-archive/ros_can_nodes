#include "MessageBuffer.h"
#include <iostream>
#include <string>
#include <mutex>
#include <condition_variable>
#include <thread>
#include "CANHelpers.hpp"

void MessageBuffer::push(const struct can_frame& data) {
    std::unique_lock<std::mutex> lk(mutex);

    q.push(data);
    
    lk.unlock();
    cv.notify_one();
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
        auto can_port = "can0";

        while (true) {
            auto data = pop();
            
            // debug exit condition
            if (data.__res0 == 8) {
                std::cout << "exit condition\n";
                break;
            }

            std::cout << data.can_id << ":";
            for (int i = 0;i < data.can_dlc;++i) {
                std::cout << data.data[i];
            }
            std::cout << "\n";

            if (CANHelpers::open_can_port(can_port) < 0) {
                std::cerr << "send failed: can port \"" << can_port << "\" open failed\n";
                continue;
            }

            if (CANHelpers::send_can_port(&data) < 0) {
                std::cerr << "send failed: frame could not be sent\n";
                CANHelpers::close_can_port();
                continue;
            }

            CANHelpers::close_can_port();
        }
    }};
    sender.detach();
}