#include "RosCanNode.h"
#include "network.h"

// main has to be in the global namespace lol
int main() {
    using namespace roscan;
    network::init();

    std::cout << "Creating new node\n";
    RosCanNodePtr node;
    node.reset(new RosCanNode("mynode1"));
    std::cout << "Created node \"" << node->getName() << "\"\n";
    node->start();
    std::cout << node->xmlrpc_manager()->getServerURI() << std::endl;

    // mini shell for testing lel
    while (1) {
        std::cout << "$ ";
        std::string s;
        getline(std::cin, s);
        if (s == "exit") {
            break;
        } else if (s == "checkMaster") {
            std::cout << node->xmlrpc_manager()->checkMaster() << std::endl;
        } else if (s == "getMasterURI") {
            std::cout << node->xmlrpc_manager()->getMasterURI() << std::endl;
        } else if (s == "getAllNodes") {
            std::vector<std::string> nodes;
            if (node->xmlrpc_manager()->getAllNodes(nodes)) {
                for (std::string& node : nodes) {
                    std::cout << node << std::endl;
                }
            } else {
                std::cout << "getAllNodes() failed!" << std::endl;
            }
        } else if (s == "getAllTopics") {
            std::cout << "subgraph: ";
            std::string subgraph;
            getline(std::cin, subgraph);
            std::vector<XMLRPCManager::TopicInfo> topics;
            if (node->xmlrpc_manager()->getAllTopics(subgraph, topics)) {
                for (XMLRPCManager::TopicInfo& topic : topics) {
                    std::cout << topic.name << " (" << topic.datatype << ")" << std::endl;
                }
            } else {
                std::cout << "getAllTopics() failed!" << std::endl;
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
