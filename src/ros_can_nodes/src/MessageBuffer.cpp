#include "MessageBuffer.hpp"
#include "CANHelpers.hpp"
#include <iostream>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>

MessageBuffer& MessageBuffer::instance() {
    static MessageBuffer instance;
    return instance;
}

void MessageBuffer::push(const can_frame& frame) {
    std::unique_lock<std::mutex> lock{mutex};

    q.push(frame);
    
    lock.unlock();
    cv.notify_one();
}

MessageBuffer::MessageBuffer() {
    std::thread sender{[this]() {
        while (true) {
            std::unique_lock<std::mutex> lock{mutex};
            cv.wait(lock, [this](){ return q.size() > 0; });

            const auto frame = q.front();
            q.pop();
            
            lock.unlock();
            
            // debug exit condition
            if (frame.__res0 == 8) {
                std::cout << "exit condition\n";
                break;
            }

            // printf("Sending header = %#08X, length = %d, data = ", frame.can_id, frame.can_dlc);
            // for (int i = 0; i < frame.can_dlc;++i) {
            //    printf("%02X ", frame.data[i]);
            // }
            // printf("\n");

            // CAN port is assumed to be open
            if (CANHelpers::send_frame(frame) < 0) {
            	std::cerr << "send failed: frame could not be sent\n";
            }
        }
    }};
    sender.detach();
}
