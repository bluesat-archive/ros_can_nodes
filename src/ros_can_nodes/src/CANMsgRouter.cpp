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
#include <sstream>
#include "CANMsgRouter.hpp"
#include "ROSCANConstants.hpp"
#include "TopicBuffers.hpp"
#include "CANHelpers.hpp"
#include "network.h"
#include "RosCanNode.hpp"
#include "RosCanNodeManager.hpp"


int main(int argc, char **argv) {
    roscan::network::init();

    CANMsgRouter::init();

    //CANMsgRouter::subscriberTest();

    CANMsgRouter::run();
}

void CANMsgRouter::init() {
    // TODO: either fail on bad open_port OR have reconnect policy
    int err = CANHelpers::open_can_port("vcan0");

    if (err) {
        throw "Failed to acuire can socket, exiting";
    }

    //TopicBuffers::instance().initBuffers();
}

void CANMsgRouter::run() {
    can_frame can_msg;

    while (1) {
        // Check for Messages
        // TODO add reconnection ability
        if (CANHelpers::read_can_port(can_msg) >= 0) {
            std::cout << "read can msg\n";
            // Pass to processCANMsg
            CANMsgRouter::processCANMsg(can_msg);
        }
    }
}

void CANMsgRouter::subscriberTest() {
    uint32_t nodeId = RosCanNodeManager::instance().registerNode("testNode", 0);

    roscan::RosCanNodePtr node = RosCanNodeManager::instance().getNode(nodeId);

    node->registerSubscriber("/front_left_wheel_axel_controller/command", "blah");
    node->registerSubscriber("/back_left_wheel_axel_controller/command", "blah");
    node->registerSubscriber("/front_left_swerve_controller/command", "blah");
    node->registerSubscriber("/back_left_swerve_controller/command", "blah");

    while (1) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void CANMsgRouter::processCANMsg(const can_frame& msg) {
    // Check the CAN Msg Header to perform routing
    uint32_t msg_header = msg.can_id & CAN_ERR_MASK;
    printf("header %#X\n", msg_header);

    if ((msg_header & ROSCANConstants::Common::bitmask_mode) == 0) {
            // Out-of-Channel communication handling
            // Currently unimplemented
            std::cout << "out of channel unimplemented\n";
    } else {
        // Check the function
        uint8_t msg_function = ((msg_header & ROSCANConstants::Common::bitmask_func) >> ROSCANConstants::Common::bitshift_func);

        switch (msg_function) {
            case ROSCANConstants::Common::ROS_TOPIC:
            {
                // Start thread to handle a ros message packet
                // Mutex is placed within handler on a per-node basis, so that
                // no 2 topics from a single node can be concurrently handling
                std::cout << "function ros_topic\n";
                CANMsgRouter::routePublishMsg(msg);
                break;
            }
            case ROSCANConstants::Common::ROS_SERVICE:
            {
                // ROS_SERVICE message handling
                // Currently unimplemented
                break;
            }
            case ROSCANConstants::Common::CONTROL:
            {
                // Assume control messages are valid and intended for main controller
                // Start a thread to handle control message.
                // TODO: handle shared resource with topicThread (nodeList, topics etc)
                std::cout << "function control\n";
                CANMsgRouter::routeControlMsg(msg);
                break;
            }
            default:
            {
                // -- Reserved --
                std::cout << "function reserved\n";
                break;
            }
        }
    }
}

// Function to cut the control msg into its components, and then call the
// appropriate Node function.
void CANMsgRouter::routeControlMsg(const can_frame& msg) {
    uint32_t mode = ((msg.can_id & ROSCANConstants::Control::bitmask_mode) >> ROSCANConstants::Control::bitshift_mode);

    std::cout << "mode = " << mode << "\n";

    switch (mode) {
        case ROSCANConstants::Control::REGISTER_NODE:
        {
            uint8_t step = ((msg.can_id & ROSCANConstants::Control::bitmask_mode0_step) >> ROSCANConstants::Control::bitshift_mode0_step);
            uint8_t hashName = ((msg.can_id & ROSCANConstants::Control::bitmask_mode0_hash) >> ROSCANConstants::Control::bitshift_mode0_hash);

            printf("step %u, hashname %X\n", step, hashName);

            char *name_buf[CAN_MAX_DLEN + 1] = {0};
            memcpy(name_buf, msg.data, msg.can_dlc);

            std::string name{(const char *)name_buf};
            std::cout << "registering \"" << name << "\"\n";

            int nodeid = RosCanNodeManager::instance().registerNode(name, hashName);
            if (nodeid < 0) {
                std::cout << "no available ids\n";
            } else {
                std::cout << "new node id " << nodeid << "\n";
            }
            break;
        }
        case ROSCANConstants::Control::DEREGISTER_NODE:
        {
            uint8_t nodeID = ((msg.can_id & ROSCANConstants::Control::bitmask_nid) >> ROSCANConstants::Control::bitshift_nid);
            std::cout << "deregistering node id " << (int)nodeID << "\n";
            RosCanNodeManager::instance().deregisterNode(nodeID);
            break;
        }
        case ROSCANConstants::Control::SUBSCRIBE_TOPIC:
        {
            //TODO: get data content for function call
            // extract node id
            uint8_t nodeID = ((msg.can_id & ROSCANConstants::Control::bitmask_nid) >> ROSCANConstants::Control::bitshift_nid);
            std::cout << "subscribing for node id " << (int)nodeID << "\n";

            std::string topic, topic_type;
            extractTopic(msg, topic, topic_type);
            int topicID = RosCanNodeManager::instance().getNode(nodeID)->registerSubscriber(topic, topic_type);
            if (topicID < 0) {
                std::cout << "subscribe register failed\n";
            } else {
                std::cout << "subscription of topic \"" << topic << "\" assigned to topic id " << topicID << " of node id " << (int)nodeID << "\n";
            }
            break;
        }
        case ROSCANConstants::Control::UNREGISTER_TOPIC:
        {
            uint8_t nodeID = ((msg.can_id & ROSCANConstants::Control::bitmask_nid) >> ROSCANConstants::Control::bitshift_nid);
            uint8_t topicID = ((msg.can_id & ROSCANConstants::Control::bitmask_topic_id) >> ROSCANConstants::Control::bitshift_topic_id);
            RosCanNodeManager::instance().getNode(nodeID)->unregisterSubscriber(topicID);
            break;
        }
        case ROSCANConstants::Control::ADVERTISE_TOPIC:
        {
            uint8_t nodeID = ((msg.can_id & ROSCANConstants::Control::bitmask_nid) >> ROSCANConstants::Control::bitshift_nid);
            std::cout << "advertising for node id " << (int)nodeID << "\n";

            std::string topic, topic_type;
            extractTopic(msg, topic, topic_type);
            int topicID = RosCanNodeManager::instance().getNode(nodeID)->advertiseTopic(topic, topic_type);
            if (topicID < 0) {
                std::cout << "advertise register failed\n";
            } else {
                std::cout << "advertisement of topic \"" << topic << "\" assigned to topic id " << topicID << " of node id " << (int)nodeID << "\n";
            }
            break;
        }
        case ROSCANConstants::Control::UNREGISTER_PUBLISHER:
        {
            uint8_t nodeID = ((msg.can_id & ROSCANConstants::Control::bitmask_nid) >> ROSCANConstants::Control::bitshift_nid);
            uint8_t topicID = ((msg.can_id & ROSCANConstants::Control::bitmask_topic_id) >> ROSCANConstants::Control::bitshift_topic_id);
            RosCanNodeManager::instance().getNode(nodeID)->unregisterPublisher(topicID);
            break;
        }
        case ROSCANConstants::Control::ADVERTISE_SERVICE:
        {
            uint8_t nodeID = ((msg.can_id & ROSCANConstants::Control::bitmask_nid) >> ROSCANConstants::Control::bitshift_nid);
            //roscan::RosCanNode::getNode(nodeID)->advertiseService(std::string service);
            break;
        }
        case ROSCANConstants::Control::UNREGISTER_SERVICE:
        {
            uint8_t nodeID = ((msg.can_id & ROSCANConstants::Control::bitmask_nid) >> ROSCANConstants::Control::bitshift_nid);
            //roscan::RosCanNode::getNode(nodeID)->unregisterService(std::string service);
            break;
        }
        case ROSCANConstants::Control::PARAMETERS:
        {
            //TODO: later
            break;
        }
        case ROSCANConstants::Control::HEARTBEAT:
        {
            uint8_t nodeID = ((msg.can_id & ROSCANConstants::Control::bitmask_nid) >> ROSCANConstants::Control::bitshift_nid);
            //roscan::RosCanNode::getNode(nodeID)->heartbeat(void);
            break;
        }
        case ROSCANConstants::Control::EXTENDED:
        {
            //TODO: later
            break;
        }
        default:
        {
            //NOTE: not a valid mode
            break;
        }
    }
}

void CANMsgRouter::routePublishMsg(const can_frame& msg) {
    uint32_t msg_header = msg.can_id & CAN_ERR_MASK;

    // Check if we have reached the last expected msg
    bool last_msg = (msg.can_dlc < 8) ? false : true;

    // Grab attributes needed for accessing the buffer
    uint8_t topic = (msg_header >> ROSCANConstants::ROSTopic::bitshift_topic_id) & ROSCANConstants::ROSTopic::bitmask_topic_id;
    uint8_t nid = (msg_header >> ROSCANConstants::ROSTopic::bitshift_nid) & ROSCANConstants::ROSTopic::bitmask_nid;
    uint8_t seq = (msg_header >> ROSCANConstants::Common::bitshift_seq) & ROSCANConstants::Common::bitmask_seq;

    // Special case for messages that are made up of 7 or more can packets
    if (seq == 7) {
        seq = (msg_header >> ROSCANConstants::ROSTopic::bitshift_len) & ROSCANConstants::ROSTopic::bitmask_len;
    }

    // Key will be concatenation of topicid, and nid
    short key = (topic << 5) | nid;

    TopicBuffers::instance().processData(key, msg.data, seq, last_msg);
}

void CANMsgRouter::extractTopic(const can_frame& first, std::string& topic, std::string& topic_type) {
    std::vector<uint8_t> buf;
    std::cout << "entered topic buffering area\n";

    // extract total number of frames to wait for
    uint8_t len = ((first.can_id & ROSCANConstants::Control::bitmask_len) >> ROSCANConstants::Control::bitshift_len);
    std::cout << "waiting for " << (int)len << " total frames\n";
    buf.insert(buf.end(), first.data, first.data + first.can_dlc);

    // read until we have 'len' frames worth of data
    can_frame msg;
    for (int i = 1;i < len;++i) {
        while (1) {
            if (CANHelpers::read_can_port(msg) >= 0) {
                buf.insert(buf.end(), msg.data, msg.data + msg.can_dlc);
                break;
            }
        }
    }

    // extract topic and topic type by finding the position of the middle null char
    const auto null_char_it = std::find(buf.begin(), buf.end(), 0);
    if (null_char_it != buf.end()) {
        topic = std::string{buf.begin(), null_char_it};
        topic_type = std::string{null_char_it+1, buf.end()};
        std::cout << "received topic \"" << topic << "\" with type \"" << topic_type << "\"\n";
    } else {
        std::cout << "invalid topic/type data\n";
    }
}