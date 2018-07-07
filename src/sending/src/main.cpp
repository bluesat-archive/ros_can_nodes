#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <random>
#include <algorithm>
#include <cstring>
#include <linux/can.h>
#include "MessageBuffer.h"

constexpr int items = 3;
constexpr int num_pushers = 5;

MessageBuffer& mb = MessageBuffer::instance();

int main(int argc, char *argv[]) {
    std::vector<std::thread> pushers;

    auto pusher_func = []() {
        std::stringstream ss;
        ss << std::this_thread::get_id();
        std::string thread_id = ss.str();
        uint64_t id_base = std::stoull(thread_id);

        std::reverse(thread_id.begin(), thread_id.end());

        std::mt19937 gen(std::chrono::system_clock::now().time_since_epoch().count());
        std::uniform_int_distribution<uint64_t> distrib(0,500);

        for (int i = 0;i < items;++i) {
            std::string id = std::to_string(i) + " - " + thread_id;
            struct can_frame f;
            f.can_id = id_base + i;
            f.can_dlc = 8;
            f.__pad = 0;
            f.__res0 = 0;
            f.__res1 = 0;
            memcpy(f.data, id.c_str(), f.can_dlc);
            mb.push(f);

            std::this_thread::sleep_for(std::chrono::milliseconds(distrib(gen)));
        }
    };
    
    for (int i = 0;i < num_pushers;++i) {
        pushers.push_back(std::thread{pusher_func});
    }

    for (int i = 0;i < num_pushers;++i) {
        pushers[i].join();
    }

    struct can_frame f;
    f.can_id = 0;
    f.can_dlc = 0;
    f.__pad = 0;
    f.__res0 = 8;
    f.__res1 = 0;

    mb.push(f);

    return 0;
}