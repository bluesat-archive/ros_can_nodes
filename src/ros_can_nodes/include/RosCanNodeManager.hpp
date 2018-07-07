#ifndef ROSCANNODEMANAGER_H
#define ROSCANNODEMANAGER_H

#include <string>
#include <cstdint>
#include "RosCanNode.hpp"
#include <boost/thread/mutex.hpp>

class RosCanNodeManager {
    public:
        static RosCanNodeManager& instance();

        roscan::RosCanNodePtr getNode(uint8_t id);
        uint8_t registerNode(std::string& name, uint8_t hashName);
        void deregisterNode(uint8_t id);

    private:
        RosCanNodeManager() {}

        RosCanNodeManager(const RosCanNodeManager&) = delete;
        void operator=(const RosCanNodeManager&) = delete;

        boost::mutex nodeListMutex;
        roscan::RosCanNodePtr nodeList[MAX_NODES];
};

#endif // ROSCANNODEMANAGER_H
