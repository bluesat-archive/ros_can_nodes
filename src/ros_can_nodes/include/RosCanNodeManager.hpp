#ifndef ROSCANNODEMANAGER_H
#define ROSCANNODEMANAGER_H

#include <string>
#include <cstdint>
#include "RosCanNode.hpp"
#include <mutex>

#define MAX_NODES 16

class RosCanNodeManager {
    public:
        static RosCanNodeManager& instance();

        roscan::RosCanNodePtr getNode(const uint8_t id);
        int registerNode(const std::string& name, const uint8_t hashName, const int request_nid = -1);
        void deregisterNode(const uint8_t id);

    private:
        RosCanNodeManager() {}

        RosCanNodeManager(const RosCanNodeManager&) = delete;
        void operator=(const RosCanNodeManager&) = delete;

        std::mutex nodeListMutex;
        roscan::RosCanNodePtr nodeList[MAX_NODES];
};

#endif // ROSCANNODEMANAGER_H
