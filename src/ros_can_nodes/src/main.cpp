#include "RosCanNode.h"
#include "network.h"
#include <ros/ros.h>
#include <csignal>
#include <algorithm>
#include <std_msgs/String.h>
#include <thread>
#include <chrono>

#define TOPIC "/CAN_test"

using namespace roscan;

//std::vector<RosCanNodePtr> nodes{2};

volatile bool quit{false};
std::thread publisher_thread;
std::thread subscriber_thread;

void sub_callback_string(const std_msgs::String::ConstPtr& msg) {
    std::cout << "Subscriber node callback received '" << msg->data << "'\n";
}

void publisher_core() {
    RosCanNodePtr node;
    node.reset(new RosCanNode("can_pub_node"));
    node->start();
    auto pub = node->advertise<std_msgs::String>(TOPIC, 10);

    std_msgs::String msg;
    int i = 0;
    ros::Rate r(1);
    while (!quit && i < 10) {
        msg.data = ("publisher string number " + std::to_string(i)).c_str();
        std::cout << "publisher node publishing '" << msg.data << "'\n";
        pub->publish(msg);
        ++i;
        r.sleep();
    }
    node->shutdown();
}

void subscriber_core() {
    RosCanNodePtr node;
    node.reset(new RosCanNode("can_sub_node"));
    node->start();

    node->subscribe(TOPIC, 1000, &sub_callback_string);

    int i = 0;
    ros::Rate r(1);
    while (!quit && i < 10) {
        node->spinOnce();
        r.sleep();
        ++i;
    }

    node->shutdown();
}

void cleanup() {
    /*for (const auto& node : nodes) {
        if (node) {
            node->shutdown();
        }
    }*/

    if (publisher_thread.joinable()) {
        publisher_thread.join();
    }
    if (subscriber_thread.joinable()) {
        subscriber_thread.join();
    }
}

void sigint_handler(int signal) {
    (void)signal;
    std::cout << "User signalled quit\n";
    quit = true;
    cleanup();
    exit(0);
}

int main() {
    signal(SIGINT, sigint_handler);
    network::init();

    publisher_thread = std::thread{publisher_core};
    subscriber_thread = std::thread{subscriber_core};
    //publisher_core();
    //subscriber_core();

    /*while (1) {
        std::cout << "$ ";
        std::string s;
        getline(std::cin, s);
        if (s == "") {
            break;
        } else if (s == "h" || s == "help") {
            std::cout << "arg1 arg2 (arg3)" << std::endl;
            std::cout << "exit by entering CTRL-D" << std::endl;
        } else {
            std::istringstream ss(s);
            std::vector<std::string> args;
            std::string arg;
            while (ss >> arg) {
                args.push_back(arg);
            }
            if (args.size() > 1) {
                int index;
                if (args[0] == "create") {
                    RosCanNodePtr node;
                    node.reset(new RosCanNode(args[1]));
                    node->start();
                    nodes.push_back(node);
                } else if (args[0] == "checkMaster") {
                    std::istringstream(args[1]) >> index;
                    std::cout << nodes[index]->xmlrpc_manager()->checkMaster() << std::endl;
                } else if (args[0] == "getMasterURI") {
                    std::istringstream(args[1]) >> index;
                    std::cout << nodes[index]->xmlrpc_manager()->getMasterURI() << std::endl;
                } else if (args[0] == "getAllNodes") {
                    std::istringstream(args[1]) >> index;
                    std::vector<std::string> nodelist;
                    if (nodes[index]->xmlrpc_manager()->getAllNodes(nodelist)) {
                        for (const auto& node : nodelist) {
                            std::cout << node << std::endl;
                        }
                    } else {
                        std::cout << "getAllNodes() failed!" << std::endl;
                    }
                } else if (args[0] == "getAllTopics") {
                    std::istringstream(args[1]) >> index;
                    std::cout << "subgraph: ";
                    std::string subgraph;
                    getline(std::cin, subgraph);
                    std::vector<XMLRPCManager::TopicInfo> topiclist;
                    if (nodes[index]->xmlrpc_manager()->getAllTopics(subgraph, topiclist)) {
                        for (const auto& topic : topiclist) {
                            std::cout << topic.name << " (" << topic.datatype << ")" << std::endl;
                        }
                    } else {
                        std::cout << "getAllTopics() failed!" << std::endl;
                    }
                }
            }
        }*/
        /*
        if (s == "subscribe") {
            ros::SubscribeOptions ops;
            ops.template init<std_msgs::String>("/chatter", 1, boost::bind(&roscan::RosCanNode::subChatterCallback, node, _1));
            ops.transport_hints = ros::TransportHints();
            node->subscribe(ops);
            while(1) {
                // spin on callbacks
                node->spinOnce();
                sleep(1);
            }
        }*/
    //}
    cleanup();
    return 0;
}
