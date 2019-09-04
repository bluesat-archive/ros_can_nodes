#include "CANSendQueue.hpp"
#include "CANHelpers.hpp"
#include <mutex>
#include <condition_variable>
#include <thread>
#include <ros/console.h>

//#define DEBUG

CANSendQueue& CANSendQueue::instance() {
    static CANSendQueue instance;
    return instance;
}

void CANSendQueue::push(const can_frame& frame) {
    std::unique_lock<std::mutex> lock{mutex};

    q.push(frame);
    
    lock.unlock();
    cv.notify_one();
}

CANSendQueue::CANSendQueue() {
    std::thread sender{[this]() {
        while (true) {
            std::unique_lock<std::mutex> lock{mutex};
            cv.wait(lock, [this](){ return q.size() > 0; });

            const auto frame = q.front();
            q.pop();
            
            lock.unlock();

            #ifdef DEBUG
            
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

            #endif // DEBUG

            // CAN port is assumed to be open
            if (CANHelpers::send_frame(frame) < 0) {
            	ROS_ERROR("send failed: frame could not be sent");
            }
        }
    }};
    sender.detach();
}
