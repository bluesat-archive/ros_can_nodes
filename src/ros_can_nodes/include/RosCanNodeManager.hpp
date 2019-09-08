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
        /**
         * @return the first unused topic id
         */
        int getFirstFreeTopic();
        /**
         * Cheecks the availability of the topic
         * @param i the index
         * @return 0 if its available.
         */
        int getTopicIdAvailability(const int i);

        /**
         * @return the total number of topics including those in use
         */
        int getTopicsSize() const;
        /**
         * Claims a specific topic id (marking it in use)
         * @param id the id to claim
         */
        void claimTopicId(const int id);

    private:
        RosCanNodeManager() {}

        RosCanNodeManager(const RosCanNodeManager&) = delete;
        void operator=(const RosCanNodeManager&) = delete;

        std::mutex nodeListMutex;
        roscan::RosCanNodePtr nodeList[MAX_NODES];
        std::bitset<MAX_TOPICS> topicIds;
};

#endif // ROSCANNODEMANAGER_H
