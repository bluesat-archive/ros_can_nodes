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
#include <stdexcept>
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
    // CANMsgRouter::publisherTest();

    CANMsgRouter::run();
}

void CANMsgRouter::init() {
    // TODO: either fail on bad open_port OR have reconnect policy
    int err = CANHelpers::open_can_port("can0");

    if (err) {
        throw std::runtime_error("Failed to acquire CAN socket, exiting");
    }
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

void CANMsgRouter::publisherTest() {
    ROS_INFO("Registering\n");

    int nodeId = RosCanNodeManager::instance().registerNode("can_publish_test", 5, 5);
    if (nodeId < 0) {
        ROS_INFO("unable to register node");
        return;
    }

    ROS_INFO("Registered***");

    roscan::RosCanNodePtr node = RosCanNodeManager::instance().getNode(nodeId);

    //node->advertiseTopic("/aaa", "std_msgs/Float64");
    node->advertiseTopic("/aaa", "owr_messages/motor");

    ROS_INFO("publishing...");

    CANMsgRouter::run();
}

void CANMsgRouter::subscriberTest() {
    ROS_INFO("Registering\n");

    int nodeId1 = RosCanNodeManager::instance().registerNode("left_loco", 2, 2);
    int nodeId2 = RosCanNodeManager::instance().registerNode("right_loco", 3, 3);
    int nodeId4 = RosCanNodeManager::instance().registerNode("arm", 4, 4);
    int nodeId5 = RosCanNodeManager::instance().registerNode("science", 7, 7);

    if (nodeId1 < 0 || nodeId2 < 0 || nodeId4 < 0) {
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
    printf("raw header %#X\n", msg.can_id);
    uint32_t header = msg.can_id & CAN_ERR_MASK;
    printf("header %#X\n", header);

    if (ROSCANConstants::Common::mode(header) == 0) {
            // Out-of-Channel communication handling
            // Currently unimplemented
            std::cout << "out of channel unimplemented\n";
    } else {
        // Check the function
        switch (ROSCANConstants::Common::func(header)) {
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
    uint8_t mode = ROSCANConstants::Control::mode(msg.can_id);
    std::cout << "mode = " << (int)mode << "\n";

    switch (mode) {
        case ROSCANConstants::Control::REGISTER_NODE:
        {
            uint8_t step = ROSCANConstants::Control::mode0_step(msg.can_id);
            uint8_t hashName = ROSCANConstants::Control::mode0_hash(msg.can_id);

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
            // send message back, everything is the same apart from the step
            can_frame response = msg;
	    ROSCANConstants::Control::step_insert(response.can_id, step+1);
	    printf("sending header %x\n", response.can_id); 
	    response.can_dlc = 4;
	    response.data[1] = nodeid;
	    
	    CANHelpers::send_can_port(response);
            
            break;
        }
        case ROSCANConstants::Control::DEREGISTER_NODE:
        {
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            std::cout << "deregistering node id " << (int)nodeID << "\n";
            RosCanNodeManager::instance().deregisterNode(nodeID);
            break;
        }
        case ROSCANConstants::Control::SUBSCRIBE_TOPIC:
        {
            //TODO: get data content for function call
            // extract node id
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
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
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            uint8_t topicID = ROSCANConstants::Control::topic_id(msg.can_id);
            RosCanNodeManager::instance().getNode(nodeID)->unregisterSubscriber(topicID);
            break;
        }
        case ROSCANConstants::Control::ADVERTISE_TOPIC:
        {
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
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
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            uint8_t topicID = ROSCANConstants::Control::topic_id(msg.can_id);
            RosCanNodeManager::instance().getNode(nodeID)->unregisterPublisher(topicID);
            break;
        }
        case ROSCANConstants::Control::ADVERTISE_SERVICE:
        {
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            //roscan::RosCanNode::getNode(nodeID)->advertiseService(std::string service);
            break;
        }
        case ROSCANConstants::Control::UNREGISTER_SERVICE:
        {
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
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
            uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
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

    printf("publish frame: node id %d, topic id %d, message len %d, seq %d\n", nodeID, topicID, len, seq);

    // Key will be concatenation of topicid, and nid
    short key = (topicID << 4) | nodeID;

    if (seq == 0) {
        TopicBuffers::instance().reset(key, len);
    }

    if (TopicBuffers::instance().append(key, msg.data, msg.can_dlc)) {
        const auto& buf = TopicBuffers::instance().get(key);
        printf("topic %d buf complete:", topicID);
        for (const auto b : buf) {
            printf(" %02x", b);
        }
        printf("\n");
        RosCanNodeManager::instance().getNode(nodeID)->publish(topicID, buf);
    }
}

void CANMsgRouter::extractTopic(const can_frame& first, std::string& topic, std::string& topic_type) {
    std::vector<uint8_t> buf;
    std::cout << "entered topic buffering area\n";

    // extract total number of frames to wait for
    uint8_t len = ROSCANConstants::Control::len(first.can_id);
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
    const auto null_char_it1 = std::find(buf.cbegin(), buf.cend(), 0);
    const auto null_char_it2 = std::find(null_char_it1 + 1, buf.cend(), 0);
    if (null_char_it1 != buf.cend() && null_char_it2 != buf.cend()) {
        topic = std::string{buf.cbegin(), null_char_it1};
        topic_type = std::string{null_char_it1 + 1, null_char_it2};
        std::cout << "received topic \"" << topic << "\" with type \"" << topic_type << "\"\n";
    } else {
        std::cout << "invalid topic/type data\n";
    }
}
