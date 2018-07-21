#include "RosCanNodeManager.hpp"
#include "RosCanNode.hpp"
#include <cstdint>

RosCanNodeManager& RosCanNodeManager::instance() {
    static RosCanNodeManager instance;
    return instance;
}

roscan::RosCanNodePtr RosCanNodeManager::getNode(const uint8_t id) {
    std::lock_guard<std::mutex> nodeListLock{nodeListMutex};
    return nodeList[id];
}

uint8_t RosCanNodeManager::registerNode(const std::string& name, const uint8_t hashName) {
    uint8_t index = 0;
    //get the first id

    {
        std::lock_guard<std::mutex> nodeListLock{nodeListMutex};

        while (index < MAX_NODES && nodeList[index] != nullptr) {
            ++index;
            if (index >= MAX_NODES) {
                std::cout << "No available ids\n";
                break;
            }
        }

        if (index < MAX_NODES) {
            //TODO: take name from frame data
            nodeList[index].reset(new roscan::RosCanNode{name, index});
            nodeList[index]->start();

            // if successful, create thread to loop spinOnce for any subscribers
            nodeList[index]->startSpinThread();
        }
    }

    return index;
    // TODO send response?
}

void RosCanNodeManager::deregisterNode(const uint8_t id) {
    {
        std::lock_guard<std::mutex> nodeListLock{nodeListMutex};
        nodeList[id]->shutdown();
        nodeList[id].reset();
    }

    //TODO send response, will need to store nodeid if so.
}
