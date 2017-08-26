#include"main.h"
#include"xmlrpc_manager.h"
#include"network.h"

RosCanNode::RosCanNode() {
    xmlrpcManager.reset(new roscan::XMLRPCManager);
    xmlrpcManager->start();
}

RosCanNode::~RosCanNode() {
    xmlrpcManager->shutdown();
}

int main() {

    roscan::network::init();

    RosCanNode *node = new RosCanNode();
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
