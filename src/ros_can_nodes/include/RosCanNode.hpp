/**
 * Date Started:
 * Original Author: Nuno Das Neves
 * Editors: Yiwei Han
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose: Extension of RosNode with functions to interact with the CAN bus.
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2018
 */

#ifndef ROSCANNODE_H
#define ROSCANNODE_H

#include "ros_node_lib/RosNode.hpp"
#include <thread>
#include <bitset>
#include <unordered_map>
#include <mutex>
#include <utility>
#include <boost/shared_ptr.hpp>
#include "shape_shifter.hpp"

#define MAX_TOPICS 128

namespace roscan {

    class RosCanNode;
    typedef boost::shared_ptr<RosCanNode> RosCanNodePtr;

    class RosCanNode : public RosNode {
        public:
            RosCanNode(const std::string& name, const uint8_t id = 0);

            uint8_t getID() const { return id_; }

            // ==================================================
            //           CAN-facing methods and attributes
            // ==================================================

            void deregisterNode();
            void heartbeat();
            int registerSubscriber(const std::string& topic, const std::string& topic_type, const int request_tid = -1);
            int unregisterSubscriber(const uint8_t topic);
            PublisherPtr make_publisher(const std::string& topic, const std::string& topic_type);
            int advertiseTopic(const std::string& topic, const std::string& topic_type, const int request_tid = -1);
            int unregisterPublisher(const uint8_t topic);
            void publish(const uint8_t topicID, const std::vector<uint8_t>& value);
            int setParam(std::string key);
            int deleteParam(std::string key);
            int advertiseService(std::string service);
            int unregisterService(std::string service);
            int searchParam(std::string key);
            int subscribeParam(std::string key);
            int unsubscribeParam(std::string key);
            int hasParam(std::string key);
            int getParamNames();
            int getParam(std::string key);

            template<typename M>
            M convert_buf(const std::vector<uint8_t>& buf) {
                auto value = std::vector<uint8_t>(buf.cbegin(), buf.cend());
                std::cout << "message type size " << sizeof(M) << "\n";
                std::cout << "buffer size " << value.size() << "\n";
                if (sizeof(M) > value.size()) {
                    auto sz = sizeof(M) - value.size();
                    std::cout << "converting buffer: appending " << sz << " 0s to buffer\n";
                    value.insert(value.end(), sz, 0);
                }
                return *(M *)value.data();
            }

        private:
            const uint8_t id_;
            
            void rosCanCallback(const RosIntrospection::ShapeShifter::ConstPtr& msg, const uint8_t topic_num, const std::string& topic_name);

            std::mutex topicLock;
            std::bitset<MAX_TOPICS> topicIds;

            int getFirstFreeTopic();
            std::unordered_map<uint8_t, std::pair<PublisherPtr, std::string>> publishers;
    };

} // namespace roscan

#endif // ROSCANNODE_H
