#include"RosCanNode.h"
#include"network.h"
#include<boost/make_shared.hpp>

namespace roscan
{
    RosCanNode::RosCanNode(std::string name) {
        name_ = name;

        xmlrpcManager.reset(new XMLRPCManager);
        xmlrpcManager->start();

        pollManager.reset(new PollManager);
        pollManager->start();

        connectionManager.reset(new ConnectionManager);
        connectionManager->start(*this);

        RosCanNodePtr nodeptr = boost::make_shared<RosCanNode>(*this);
        topicManager.reset(new TopicManager);
        topicManager->start(nodeptr);

    }

    RosCanNode::~RosCanNode() {
        xmlrpcManager->shutdown();
    }
}

using namespace roscan;

int main() {

    network::init();

    RosCanNode *node = new RosCanNode("bob");
    std::cout << node->xmlrpcManager->getServerURI() << std::endl;

    // mini shell for testing lel
    while (1) {
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
