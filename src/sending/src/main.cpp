#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <random>
#include "MessageBuffer.h"

constexpr int items = 10;
constexpr int num_pushers = 10;

MessageBuffer mb;

int main(int argc, char *argv[]) {
    std::vector<std::thread> pushers;

    auto pusher_func = []() {
        std::mt19937 gen(std::chrono::system_clock::now().time_since_epoch().count());
        std::uniform_int_distribution<int64_t> distrib(0,500);

        for (int i = 0;i < items;++i) {
            auto thread_id = std::this_thread::get_id();
            std::stringstream ss;
            ss << thread_id;

            mb.push(ss.str() + " - " + std::to_string(i));
            std::this_thread::sleep_for(std::chrono::milliseconds(distrib(gen)));
        }
    };

    mb.startSendThread();
    
    for (int i = 0;i < num_pushers;++i) {
        pushers.push_back(std::thread{pusher_func});
    }

    for (int i = 0;i < num_pushers;++i) {
        pushers[i].join();
    }

    mb.push("");

    return 0;
}