#include"RosCanNode.h"
#include"network.h"
#include<boost/make_shared.hpp>

namespace roscan
{
    RosCanNode::RosCanNode(std::string name) {
        name_ = name;

        std::cout << "  Creating xmlrpc manager\n";
        xmlrpcManager.reset(new XMLRPCManager);

        std::cout << "  Creating poll manager\n";
        pollManager.reset(new PollManager);
        std::cout << "  Starting poll manager\n";
        pollManager->start();

        std::cout << "  Creating connection manager\n";
        connectionManager.reset(new ConnectionManager);
        std::cout << "  Starting connection manager\n";
        connectionManager->start(*this);

        std::cout << "  Creating topic manager\n";
        RosCanNodePtr nodeptr = boost::make_shared<RosCanNode>(*this);
        topicManager.reset(new TopicManager);
        std::cout << "  Starting topic manager\n";
        topicManager->start(nodeptr);
        std::cout << "  Done!\n";

        // xmlrpc manager must be started _after_ all functions are bound to it
        std::cout << "  Starting xmlrpc manager\n";
        xmlrpcManager->start();

    }

    RosCanNode::~RosCanNode() {
        xmlrpcManager->shutdown();
    }
}

using namespace roscan;

int main() {

    network::init();

    std::cout << "Creating new node\n";
    RosCanNode *node = new RosCanNode("bob");
    std::cout << node->xmlrpcManager->getServerURI() << std::endl;

    // mini shell for testing lel
    while (1) {
        std::cout << "$ ";
        std::string s;
        std::cin >> s;
        if (s == "exit") break;
        if (s == "checkMaster") {
            std:: cout << node->xmlrpcManager->checkMaster("yo") << std::endl;
        }
        if (s == "getMasterURI") {
            std:: cout << node->xmlrpcManager->getMasterURI() << std::endl;
        }
        if (s == "getAllNodes") {
            std::vector<std::string> nodes;
            if (node->xmlrpcManager->getAllNodes("yo", nodes)) {
                for (unsigned i = 0; i < nodes.size(); ++i) {
                    std::cout << nodes[i] << std::endl;
                }

            } else {
                std::cout << "getAllNodes() failed!" << std::endl;
            }
        }
    }

    delete node;
    return 0;
}
