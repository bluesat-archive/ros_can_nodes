#include "MessageBuffer.hpp"
#include "CANHelpers.hpp"
#include <iostream>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <ros/console.h>

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
                ROS_INFO("exit condition");
                break;
            }

            char str[1000] = {0};
            sprintf(str, "Sending header = %#08X, length = %d, data:", frame.can_id, frame.can_dlc);
            for (int i = 0; i < frame.can_dlc;++i) {
                sprintf(str, "%s %02x", str, frame.data[i]);
            }
            ROS_DEBUG("%s", str);

            // CAN port is assumed to be open
            if (CANHelpers::send_frame(frame) < 0) {
            	ROS_ERROR("send failed: frame could not be sent");
            }
        }
    }};
    sender.detach();
}
