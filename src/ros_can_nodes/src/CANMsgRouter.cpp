/*
 * Date Started: 29/09/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */

#include <linux/can.h>
#include <thread>
#include <chrono>
#include <string>
#include <stdexcept>
#include "IntrospectionHelpers.hpp"
#include "CANMsgRouter.hpp"
#include "ROSCANConstants.hpp"
#include "TopicBuffers.hpp"
#include "CANHelpers.hpp"
#include "RosCanNode.hpp"
#include "RosCanNodeManager.hpp"
#include "message_properties_map.hpp"

int main(int argc, char **argv) {
    CANMsgRouter::init();

    //CANMsgRouter::subscriberTest();
    //CANMsgRouter::publisherTest();

    CANMsgRouter::run();
}

void CANMsgRouter::init() {
    // preload registering of all possible messages
    for (const auto& m: message_properties_map) {
        IntrospectionHelpers::register_message(m.first, m.second.definition);
    }

    // TODO: either fail on bad open_port OR have reconnect policy
    int err = CANHelpers::open_port("can0");

    if (err) {
        throw std::runtime_error("Failed to acquire CAN socket, exiting");
    }
}

void CANMsgRouter::run() {
    can_frame can_msg;

    while (1) {
        // Check for Messages
        // TODO add reconnection ability
        if (CANHelpers::read_frame(can_msg) >= 0) {
            ROS_INFO("received can msg");
            // Pass to processCANMsg
            CANMsgRouter::processCANMsg(can_msg);
        }
    }
}

void CANMsgRouter::publisherTest() {
    ROS_INFO("Registering");

    int nodeId = RosCanNodeManager::instance().registerNode("can_publish_test", 5);
    if (nodeId < 0) {
        ROS_INFO("unable to register node");
        return;
    }

    ROS_INFO("Registered***");

    roscan::RosCanNodePtr node = RosCanNodeManager::instance().getNode(nodeId);

    node->advertiseTopic("/aaa", "owr_messages/tester");

    ROS_INFO("publishing...");

    CANMsgRouter::run();
}

void CANMsgRouter::subscriberTest() {
    ROS_INFO("Registering");

    int nodeId1 = RosCanNodeManager::instance().registerNode("left_loco", 2, 2);
    int nodeId2 = RosCanNodeManager::instance().registerNode("right_loco", 3, 3);
    int nodeId4 = RosCanNodeManager::instance().registerNode("arm", 4, 4);
    int nodeId5 = RosCanNodeManager::instance().registerNode("science", 7, 7);

    if (nodeId1 < 0 || nodeId2 < 0 || nodeId4 < 0 || nodeId5 < 0) {
        throw std::runtime_error("unable to register all subscriber nodes, exiting");
    }

    ROS_INFO("Registered***");

    roscan::RosCanNodePtr l_loco_node = RosCanNodeManager::instance().getNode(nodeId1);
    roscan::RosCanNodePtr r_loco_node = RosCanNodeManager::instance().getNode(nodeId2);
    roscan::RosCanNodePtr arm_node = RosCanNodeManager::instance().getNode(nodeId4);
    roscan::RosCanNodePtr science_node = RosCanNodeManager::instance().getNode(nodeId5);

    // 0
    l_loco_node->registerSubscriber("/front_left_wheel_axel_controller/command", "blah");
    // 1
    r_loco_node->registerSubscriber("/front_right_wheel_axel_controller/command", "blah");
    // 2
    l_loco_node->registerSubscriber("/back_left_wheel_axel_controller/command", "blah");
    // 3
    r_loco_node->registerSubscriber("/back_right_wheel_axel_controller/command", "blah");
    // 4
    l_loco_node->registerSubscriber("/front_left_swerve_controller/command", "blah");
    // 5
    r_loco_node->registerSubscriber("/front_right_swerve_controller/command", "blah");
    // 6
    l_loco_node->registerSubscriber("/back_left_swerve_controller/command", "blah");
    // 7
    r_loco_node->registerSubscriber("/back_right_swerve_controller/command", "blah");

    arm_node->registerSubscriber("/arm_base_rotate_controller/command", "blah");
    arm_node->registerSubscriber("/arm_top_controller/command", "blah");
    arm_node->registerSubscriber("/arm_bottom_controller/command", "blah");
    arm_node->registerSubscriber("/claw_rotate_controller/command", "blah");
    arm_node->registerSubscriber("/claw_grip_controller/command", "blah");

    science_node->registerSubscriber("/science/request", "blah", 13);
    science_node->advertiseTopic("/science/data", "owr_messages/science");

    ROS_INFO("Subscribed");
    CANMsgRouter::run();
}

void CANMsgRouter::processCANMsg(const can_frame& msg) {
    // Check the CAN Msg Header to perform routing
    ROS_INFO("raw header %#X", msg.can_id);
    uint32_t header = msg.can_id & CAN_ERR_MASK;
    ROS_INFO("header %#X", header);

    if (ROSCANConstants::Common::mode(header) == 0) {
            // Out-of-Channel communication handling
            // Currently unimplemented
            ROS_INFO("out of channel - unimplemented");
    } else {
        // Check the function
        switch (ROSCANConstants::Common::func(header)) {
            case ROSCANConstants::Common::ROS_TOPIC:
            {
                // Start thread to handle a ros message packet
                // Mutex is placed within handler on a per-node basis, so that
                // no 2 topics from a single node can be concurrently handling
                ROS_INFO("function is ROS_TOPIC");
                CANMsgRouter::routePublishMsg(msg);
                break;
            }
            case ROSCANConstants::Common::ROS_SERVICE:
            {
                // ROS_SERVICE message handling
                // Currently unimplemented
                ROS_INFO("function is ROS_SERVICE - unimplemented");
                break;
            }
            case ROSCANConstants::Common::CONTROL:
            {
                // Assume control messages are valid and intended for main controller
                // Start a thread to handle control message.
                // TODO: handle shared resource with topicThread (nodeList, topics etc)
                ROS_INFO("function is CONTROL");
                CANMsgRouter::routeControlMsg(msg);
                break;
            }
            default:
            {
                ROS_INFO("function is RESERVED - %d", ROSCANConstants::Common::func(header));
                break;
            }
        }
    }
}

// Function to cut the control msg into its components, and then call the
// appropriate Node function.
void CANMsgRouter::routeControlMsg(const can_frame& msg) {
    uint8_t mode = ROSCANConstants::Control::mode(msg.can_id);
    ROS_INFO("mode = %d", mode);

    switch (mode) {
        case ROSCANConstants::Control::REGISTER_NODE:
        {
            uint8_t step = ROSCANConstants::Control::mode0_step(msg.can_id);
            uint8_t hashName = ROSCANConstants::Control::mode0_hash(msg.can_id);

            ROS_INFO("step %u, hashname %X", step, hashName);

            char *name_buf[CAN_MAX_DLEN + 1] = {0};
            memcpy(name_buf, msg.data, msg.can_dlc);

            std::string name{(const char *)name_buf};
            ROS_INFO("registering \"%s\"", name.c_str());

            int nodeID = RosCanNodeManager::instance().registerNode(name, hashName);
            if (nodeID < 0) {
                ROS_INFO("no available node ids");
            } else {
                ROS_INFO("new node id %d", nodeID);
            }
            // send message back, everything is the same apart from the step
            can_frame response = msg;
            ROSCANConstants::Control::mode0_step_insert(response.can_id, step+1);
            ROS_INFO("sending header %x", response.can_id);
            response.can_dlc = 4;
            response.data[0] = nodeID;

            CANHelpers::send_frame(response);
            break;
        }
        case ROSCANConstants::Control::DEREGISTER_NODE:
        {
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            ROS_INFO("deregistering node id %d", nodeID);
            RosCanNodeManager::instance().deregisterNode(nodeID);
            break;
        }
        case ROSCANConstants::Control::SUBSCRIBE_TOPIC:
        {
            //TODO: get data content for function call
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            ROS_INFO("subscribing for node id %d", nodeID);

            std::string topic, topic_type;
            extractTopic(msg, topic, topic_type);
            int topicID = RosCanNodeManager::instance().getNode(nodeID)->registerSubscriber(topic, topic_type);
            if (topicID < 0) {
                ROS_INFO("subscribe register failed");
            } else {
                ROS_INFO("subscription of topic \"%s\" assigned to topic id %d of node id %d", topic.c_str(), topicID, nodeID);
                can_frame response = msg;
                ROSCANConstants::Control::step_insert(response.can_id, 1);
                ROS_INFO("sending header %x", response.can_id);
                response.can_dlc = 4;
                response.data[0] = topicID;

                CANHelpers::send_frame(response);
            }
            break;
        }
        case ROSCANConstants::Control::UNREGISTER_TOPIC:
        {
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            uint8_t topicID = ROSCANConstants::Control::topic_id(msg.can_id);
            RosCanNodeManager::instance().getNode(nodeID)->unregisterSubscriber(topicID);
            break;
        }
        case ROSCANConstants::Control::ADVERTISE_TOPIC:
        {
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            ROS_INFO("advertising for node id %d", nodeID);

            std::string topic, topic_type;
            extractTopic(msg, topic, topic_type);
            int topicID = RosCanNodeManager::instance().getNode(nodeID)->advertiseTopic(topic, topic_type);
            if (topicID < 0) {
                ROS_INFO("advertise register failed");
            } else {
                ROS_INFO("advertisement of topic \"%s\" assigned to topic id %d of node id %d", topic.c_str(), topicID, nodeID);
                can_frame response = msg;
                ROSCANConstants::Control::step_insert(response.can_id, 1);
                ROS_INFO("sending header %x", response.can_id);
                response.can_dlc = 4;
                response.data[0] = topicID;

                CANHelpers::send_frame(response);
            }
            break;
        }
        case ROSCANConstants::Control::UNREGISTER_PUBLISHER:
        {
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            uint8_t topicID = ROSCANConstants::Control::topic_id(msg.can_id);
            RosCanNodeManager::instance().getNode(nodeID)->unregisterPublisher(topicID);
            break;
        }
        case ROSCANConstants::Control::ADVERTISE_SERVICE:
        {
            //uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            //roscan::RosCanNode::getNode(nodeID)->advertiseService(std::string service);
            ROS_INFO("mode is ADVERTISE_SERVICE - unimplemented");
            break;
        }
        case ROSCANConstants::Control::UNREGISTER_SERVICE:
        {
            //uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            //roscan::RosCanNode::getNode(nodeID)->unregisterService(std::string service);
            ROS_INFO("mode is UNREGISTER_SERVICE - unimplemented");
            break;
        }
        case ROSCANConstants::Control::PARAMETERS:
        {
            ROS_INFO("mode is PARAMETERS - unimplemented");
            break;
        }
        case ROSCANConstants::Control::HEARTBEAT:
        {
            //uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            //roscan::RosCanNode::getNode(nodeID)->heartbeat(void);
            ROS_INFO("mode is HEARTBEAT - unimplemented");
            break;
        }
        case ROSCANConstants::Control::EXTENDED:
        {
            ROS_INFO("mode is EXTENDED - unimplemented");
            break;
        }
        default:
        {
            ROS_INFO("mode is INVALID");
            break;
        }
    }
}

void CANMsgRouter::routePublishMsg(const can_frame& msg) {
    uint32_t header = msg.can_id & CAN_ERR_MASK;

    // Grab attributes needed for accessing the buffer
    uint8_t nodeID = ROSCANConstants::ROSTopic::nid(header);
    uint8_t topicID = ROSCANConstants::ROSTopic::topic_id(header);
    uint8_t len = ROSCANConstants::ROSTopic::len(header);
    uint8_t seq = ROSCANConstants::Common::seq(header);

    // Special case for messages that are made up of 7 or more can packets
    if (seq == 7) {
        seq = len;
    }

    ROS_INFO("publish frame: node id %d, topic id %d, message len %d, seq %d", nodeID, topicID, len, seq);

    // Key will be concatenation of topicid, and nid
    short key = (topicID << 4) | nodeID;

    if (seq == 0) {
        TopicBuffers::instance().reset(key, len);
    }

    if (TopicBuffers::instance().append(key, msg.data, msg.can_dlc)) {
        const auto& can_buf = TopicBuffers::instance().get(key);
        char str[1000] = {0};
        sprintf(str, "topic %d buf complete:", topicID);
        for (const auto b : can_buf) {
            sprintf(str, "%s %02x", str, b);
        }
        ROS_INFO("%s", str);
        RosCanNodeManager::instance().getNode(nodeID)->publish(topicID, can_buf);
    }
}

void CANMsgRouter::extractTopic(const can_frame& first, std::string& topic, std::string& topic_type) {
    std::vector<uint8_t> buf;
    ROS_INFO("entered topic buffering area");

    // extract total number of frames to wait for
    uint8_t len = ROSCANConstants::Control::len(first.can_id);
    ROS_INFO("waiting for %d total frames", len);
    buf.insert(buf.end(), first.data, first.data + first.can_dlc);

    // read until we have 'len' frames worth of data
    // TODO: this doesn't work with multiple nodes
    can_frame msg;
    for (int i = 1;i < len;++i) {
        while (1) {
            if (CANHelpers::read_frame(msg) >= 0) {
                buf.insert(buf.end(), msg.data, msg.data + msg.can_dlc);
                ROS_INFO("read packet at i = %d size %d", i, msg.can_dlc);
                ROS_INFO("buffer contents:");
                for (const auto c : buf) {
                    ROS_INFO("%c (%d)", c, c);
                }
                break;
            }
        }
    }

    // extract topic and topic type by finding the position of the middle null char
    ROS_INFO("%ld %ld", (long int) buf.cbegin().base(), (long int) buf.cend().base());
    const auto null_char_it1 = std::find(buf.cbegin(), buf.cend(), '\0');
    const auto null_char_it2 = std::find(null_char_it1 + 1, buf.cend(), '\0');
    if (null_char_it1 != buf.cend() && null_char_it2 <= buf.cend()) {
        topic = std::string{buf.cbegin(), null_char_it1};
        topic_type = std::string{null_char_it1 + 1, null_char_it2};
        ROS_INFO("received topic \"%s\" with type \"%s\"", topic.c_str(), topic_type.c_str());
    } else {
        ROS_INFO("invalid topic/type data: %s null at %ld", std::string{buf.cbegin(), buf.cend()}.c_str(), (long int) null_char_it1.base());
        //TODO: throw an exception or something so we don't register an empty topic
    }
}
