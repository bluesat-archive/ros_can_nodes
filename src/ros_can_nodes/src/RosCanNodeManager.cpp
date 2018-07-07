#include "RosCanNodeManager.hpp"
#include "RosCanNode.hpp"
#include <cstdint>

RosCanNodeManager& RosCanNodeManager::instance() {
    static RosCanNodeManager instance;
    return instance;
}

roscan::RosCanNode *RosCanNodeManager::getNode(uint8_t id) {
    boost::mutex::scoped_lock nodeListLock(nodeListMutex);
    return nodeList[id];
}

uint8_t RosCanNodeManager::registerNode(std::string& name, uint8_t hashName) {
    uint8_t index = 0;
    //get the first id

    {
        boost::mutex::scoped_lock nodeListLock(nodeListMutex);

        while (index < MAX_NODES && nodeList[index] != NULL) {
            index++;
            if (index >= MAX_NODES) {
                std::cout << "No available ids" << '\n';
                break;
            }
        }

        if (index < MAX_NODES) {
            //TODO: take name from frame data
            roscan::RosCanNode *node = new roscan::RosCanNode(name, index);
            nodeList[index] = node;

            // if successful, create thread to loop spinOnce for any subscribers
            //node->spinThread(boost::bind(&RosCanNode::spin, node));
        }
    }

    return index;
    // TODO send response?
}

void RosCanNodeManager::deregisterNode(uint8_t id) {
    roscan::RosCanNode *node = nullptr;
    {
        boost::mutex::scoped_lock nodeListLock(nodeListMutex);
        node = nodeList[id];
        nodeList[id] = nullptr;
    }
    delete node;

    //TODO send response, will need to store nodeid if so.
}
