#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include "MessageBuffer.h"

constexpr int items = 10;
constexpr int num_pushers = 10;
constexpr int num_senders = 10;

MessageBuffer mb;

int main(int argc, char *argv[]) {
    std::vector<std::thread> pushers;
    std::vector<std::thread> senders;

    auto pusher_func = []() {
        for (int i = 0;i < items;++i) {
            mb.push(std::to_string(i));
        }
    };

    auto sender_func = []() {
        while (true) {
            auto data = mb.send();
            if (data == "") {
                break;
            }
        }
    };
    
    for (int i = 0;i < num_pushers;++i) {
        pushers.push_back(std::thread{pusher_func});
    }
    
    for (int i = 0;i < num_senders;++i) {
        senders.push_back(std::thread{sender_func});
    }

    for (int i = 0;i < num_pushers;++i) {
        pushers[i].join();
    }
    
    for (int i = 0;i < num_senders;++i) {
        mb.push("");
    }

    for (int i = 0;i < num_senders;++i) {
        senders[i].join();
    }
    return 0;
}