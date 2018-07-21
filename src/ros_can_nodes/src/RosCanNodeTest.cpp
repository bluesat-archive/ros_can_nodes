#include "RosCanNode.hpp"
#include "network.h"
#include <csignal>
#include <cstdint>
#include <std_msgs/String.h>
#include <std_msgs/Int64MultiArray.h>
#include <vector>
#include <thread>
#include <chrono>

using namespace roscan;

volatile bool quit{false};

void sigint_handler(int signal) {
    (void)signal;
    quit = true;
}

void source() {
    using namespace std::chrono;

    auto source_thread = std::thread{[](){
        int64_t count = 0;
        const auto get_time = [](){ return high_resolution_clock::now().time_since_epoch().count(); };
        const auto new_msg = [&get_time, &count](auto& msg){
            std::vector<int64_t> data = {get_time(), count};
            msg.layout.dim.push_back(std_msgs::MultiArrayDimension{});
            msg.layout.dim[0].size = data.size();
            msg.layout.dim[0].stride = 1;
            msg.layout.dim[0].label = "msg";
            msg.data.clear();
            msg.data.insert(msg.data.end(), data.cbegin(), data.cend());
        };

        RosCanNodePtr node;
        node.reset(new RosCanNode{"test_source_node"});
        node->start();
        auto pub = node->advertise<std_msgs::Int64MultiArray>("/test_source", 10);

        std_msgs::Int64MultiArray msg;

        ros::Rate r{0.5};
        while (!quit) {
            new_msg(msg);
            std::cout << "publishing:";
            for (const auto& x: msg.data) {
                std::cout << " " << x;
            }
            std::cout << "\n";
            pub->publish(msg);
            ++count;
            r.sleep();
        }
        node->shutdown();
    }};
    source_thread.join();
}

void sink() {
    auto sink_thread = std::thread{[](){
        boost::function<void(const std_msgs::Int64MultiArray::ConstPtr&)> callback;
        callback = [](const std_msgs::Int64MultiArray::ConstPtr& msg){
            std::cout << "received:";
            for (const auto& x: msg->data) {
                std::cout << " " << x;
            }
            std::cout << "\n";
        };
        RosCanNodePtr node;
        node.reset(new RosCanNode{"test_sink_node"});
        node->start();

        node->subscribe("/test_sink", 10, callback);

        ros::Rate r{2};
        while (!quit) {
            node->spinOnce();
            r.sleep();
        }
        node->shutdown();
    }};
    sink_thread.join();
}

void mid() {
    const auto mid_func = [](const auto& name, const auto& sub_topic, const auto& pub_topic){
        RosCanNodePtr node;
        node.reset(new RosCanNode{name});
        node->start();

        auto pub = node->advertise<std_msgs::Int64MultiArray>(pub_topic, 10);

        boost::function<void(const std_msgs::Int64MultiArray::ConstPtr&)> callback;
        callback = [&pub](const std_msgs::Int64MultiArray::ConstPtr& msg){
            pub->publish(*msg);
        };

        node->subscribe(sub_topic, 10, callback);
        ros::Rate r{2};
        while (!quit) {
            node->spinOnce();
            r.sleep();
        }
        node->shutdown();
    };
    auto thread1 = std::thread{mid_func, "test_mid_node1", "/test_source", "/test_mid"};
    auto thread2 = std::thread{mid_func, "test_mid_node2", "/test_mid", "/test_sink"};
    thread1.join();
    thread2.join();
}

int main(int argc, char *argv[]) {
    signal(SIGINT, sigint_handler);
    roscan::network::init();

    if (argc == 1) {
        std::cout << "no role specified\n";
        return 0;
    }

    const auto type = std::string{argv[1]};
    if (type == "source") {
        source();
    } else if (type == "sink") {
        sink();
    } else if (type == "mid") {
        mid();
    } else {
        std::cout << "no role specified\n";
    }
    return 0;
}