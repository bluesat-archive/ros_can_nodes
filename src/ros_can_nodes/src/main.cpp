#include "RosCanNode.h"
#include "network.h"

// main has to be in the global namespace lol
int main() {
    using namespace roscan;
    network::init();

    std::vector<RosCanNodePtr> roscannodes;

    // mini shell for testing lel
    while (1) {
        std::cout << "$ ";
        std::string s;
        getline(std::cin, s);
        if (s == "exit") {
            break;
        } else if (s == "h") {
            std::cout << "arg1 arg2 (arg3)\n";
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
                    roscannodes.push_back(node);
                    node->start();
                } else if (args[0] == "checkMaster") {
                    std::istringstream(args[1]) >> index;
                    std::cout << roscannodes[index]->xmlrpc_manager()->checkMaster() << std::endl;
                } else if (args[0] == "getMasterURI") {
                    std::istringstream(args[1]) >> index;
                    std::cout << roscannodes[index]->xmlrpc_manager()->getMasterURI() << std::endl;
                } else if (args[0] == "getAllNodes") {
                    std::istringstream(args[1]) >> index;
                    std::vector<std::string> nodes;
                    if (roscannodes[index]->xmlrpc_manager()->getAllNodes(nodes)) {
                        for (std::string& node : nodes) {
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
                    std::vector<XMLRPCManager::TopicInfo> topics;
                    if (roscannodes[index]->xmlrpc_manager()->getAllTopics(subgraph, topics)) {
                        for (XMLRPCManager::TopicInfo& topic : topics) {
                            std::cout << topic.name << " (" << topic.datatype << ")" << std::endl;
                        }
                    } else {
                        std::cout << "getAllTopics() failed!" << std::endl;
                    }
                }
            }
        }
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
    }
    return 0;
}
