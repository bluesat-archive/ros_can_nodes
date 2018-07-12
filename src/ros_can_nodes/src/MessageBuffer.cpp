#include "MessageBuffer.hpp"
#include "CANHelpers.hpp"
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <thread>

MessageBuffer& MessageBuffer::instance() {
    static MessageBuffer instance;
    return instance;
}

void MessageBuffer::push(const can_frame& data) {
    std::unique_lock<std::mutex> lk(mutex);

    q.push(data);
    
    lk.unlock();
    cv.notify_one();
}

MessageBuffer::MessageBuffer() {
    std::thread sender{[this]() {
        while (true) {
            std::unique_lock<std::mutex> lk(mutex);
            cv.wait(lk, [this](){ return q.size() > 0; });

            auto data = q.front();
            q.pop();
            
            lk.unlock();
            
            // debug exit condition
            if (data.__res0 == 8) {
                std::cout << "exit condition\n";
                break;
            }

            std::cout << data.can_id << ":";
            for (const auto d : data.data) {
                std::cout << d;
            }
            std::cout << "\n";

            // CAN port is assumed to be open
            if (CANHelpers::send_can_port(&data) < 0) {
                std::cerr << "send failed: frame could not be sent\n";
            }
        }
    }};
    sender.detach();
}
