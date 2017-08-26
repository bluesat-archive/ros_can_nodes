#include"main.h"
#include"xmlrpc_manager.h"

RosCanNode::RosCanNode() {
    ros::XMLRPCManagerPtr s(new ros::XMLRPCManager);
    xmlrpcManager = s;
    xmlrpcManager->start();
}

RosCanNode::~RosCanNode() {
    xmlrpcManager->shutdown();
}

int main() {
    RosCanNode *node = new RosCanNode();
    std::cout << node->xmlrpcManager->getServerURI() << std::endl;
    delete node;
    return 0;
}
