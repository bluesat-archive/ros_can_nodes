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

int RosCanNodeManager::registerNode(const std::string& name, const uint8_t hashName, const int request_nid) {
    uint8_t index = 0;
    //get the first id

    {
        std::lock_guard<std::mutex> nodeListLock{nodeListMutex};

        if (request_nid != -1 && request_nid < MAX_NODES) {
            index = request_nid;
        } else {
            while (index < MAX_NODES && nodeList[index]) {
                ++index;
                if (index >= MAX_NODES) {
                    return -1;
                }
            }
        }

        //TODO: take name from frame data
        nodeList[index].reset(new roscan::RosCanNode{name, index});
        nodeList[index]->start();

        // if successful, create thread to loop spinOnce for any subscribers
        nodeList[index]->startSpinThread();
    }
    return index;
    // TODO send response?
}

void RosCanNodeManager::deregisterNode(const uint8_t id) {
    if (id >= MAX_NODES) {
        std::cout << "attempted to deregister invalid node id " << id << "\n";
        return;
    }
    {
        std::lock_guard<std::mutex> nodeListLock{nodeListMutex};
        if (nodeList[id]) {
            nodeList[id]->shutdown();
            nodeList[id].reset();
        } else {
            std::cout << "attempted to deregister nonexistant node id " << id << "\n";
            return;
        }
    }

    //TODO send response, will need to store nodeid if so.
}
