/*
 * Date Started: 29/09/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name: ros_can_nodes
 * ROS Package: ros_can_nodes
 * Purpose: Main entry point of ros_can_nodes. Run with:
 *          - `rosrun ros_can_nodes ros_can_nodes` to use can0 as the CAN port
 *          - `rosrun ros_can_nodes ros_can_nodes vcan0` to use vcan0 as the CAN port
 * This code is released under the BSD License. Copyright BLUEsat UNSW, 2017
 */

#include <linux/can.h>
#include <thread>
#include <chrono>
#include <string>
#include <stdexcept>
#include "CANMsgRouter.hpp"
#include "ROSCANConstants.hpp"
#include "CANBuffers.hpp"
#include "CANHelpers.hpp"
#include "RosCanNode.hpp"
#include "RosCanNodeManager.hpp"

//#define DEBUG

static constexpr auto DEFAULT_CAN_PORT = "can0";

CANBuffers CANMsgRouter::topic_register_buffers{};
CANBuffers CANMsgRouter::publish_buffers{};

int main(int argc, char **argv) {
    const auto can_port = argc > 1 ? argv[1] : DEFAULT_CAN_PORT;
    CANMsgRouter::init(can_port);

    //CANMsgRouter::subscriberTest();
    //CANMsgRouter::publisherTest();

    CANMsgRouter::run();
}

void CANMsgRouter::init(const std::string& can_port) {
    // TODO: either fail on bad open_port OR have reconnect policy
    ROS_INFO_STREAM("opening CAN port " << can_port);
    int err = CANHelpers::open_port(can_port);

    if (err) {
        throw std::runtime_error("Failed to acquire CAN socket, exiting");
    }
}

void CANMsgRouter::run() {
    can_frame can_msg;

    // we don't want any nodes we don't know about on the bus, so reset everything
    CANMsgRouter::resetAllNodes();

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
    const uint8_t mode = ROSCANConstants::Control::mode(msg.can_id);
    ROS_INFO("mode = %d", mode);

    switch (mode) {
        case ROSCANConstants::Control::REGISTER_NODE:
        {
            const uint8_t step = ROSCANConstants::Control::mode0_step(msg.can_id);
            const uint8_t hashName = ROSCANConstants::Control::mode0_hash(msg.can_id);

            ROS_INFO("step %u, hashname %X", step, hashName);

            const auto name = std::string{msg.data, msg.data + msg.can_dlc};
            ROS_INFO("registering \"%s\"", name.c_str());

            const int nodeID = RosCanNodeManager::instance().registerNode(name, hashName);
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
            ROS_INFO("mode is SUBSCRIBE_TOPIC");
            topicRegisterHelper(mode, msg);
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
            ROS_INFO("mode is ADVERTISE_TOPIC");
            topicRegisterHelper(mode, msg);
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
        case ROSCANConstants::Control::CHANNEL_CONTROL:
        {
            //uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
            //roscan::RosCanNode::getNode(nodeID)->heartbeat(void);
            ROS_INFO("mode is CHANNEL_CONTROL - unimplemented");
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
    const uint32_t header = msg.can_id & CAN_ERR_MASK;

    // Grab attributes needed for accessing the buffer
    const uint8_t nodeID = ROSCANConstants::ROSTopic::nid(header);
    const uint8_t topicID = ROSCANConstants::ROSTopic::topic_id(header);
    const uint8_t len = ROSCANConstants::ROSTopic::len(header);
    uint8_t seq = ROSCANConstants::Common::seq(header);

    // Special case for messages that are made up of 7 or more can packets
    if (seq == 7) {
        seq = len;
    }

    ROS_INFO("publish frame: node id %d, topic id %d, message len %d, seq %d", nodeID, topicID, len, seq);

    // Key will be concatenation of topicid, and nid
    const uint32_t key = (topicID << 8) | nodeID;

    if (seq == 0) {
        publish_buffers.reset(key, len);
    }

    publish_buffers.append(key, msg.data, msg.can_dlc);

    if (publish_buffers.ready(key)) {
        const auto& can_buf = publish_buffers.get(key);

        #ifdef DEBUG
        char str[1000] = {0};
        sprintf(str, "topic %d buf complete:", topicID);
        for (const auto b : can_buf) {
            sprintf(str, "%s %02x", str, b);
        }
        ROS_INFO("%s", str);
        #endif

        RosCanNodeManager::instance().getNode(nodeID)->publish(topicID, can_buf);
    }
}

std::pair<std::string, std::string> CANMsgRouter::extractTopic(const std::vector<uint8_t>& buf) {
    const auto null_char_it1 = std::find(buf.cbegin(), buf.cend(), '\0');
    const auto null_char_it2 = std::find(null_char_it1 + 1, buf.cend(), '\0');
    if (null_char_it1 != buf.cend() && null_char_it2 != buf.cend()) {
        const auto topic = std::string{buf.cbegin(), null_char_it1};
        const auto topic_type = std::string{null_char_it1 + 1, null_char_it2};
        ROS_INFO("extracted topic \"%s\" with type \"%s\"", topic.c_str(), topic_type.c_str());
        return std::make_pair(topic, topic_type);
    }
    ROS_INFO("invalid topic/type data - buffer: \"%s\"", std::string{buf.cbegin(), buf.cend()}.c_str());
    return std::make_pair("", "");
}

void CANMsgRouter::topicRegisterHelper(const uint8_t mode, const can_frame& msg) {
    const uint8_t nodeID = ROSCANConstants::Control::nid(msg.can_id);
    const uint8_t len = ROSCANConstants::Control::len(msg.can_id);
    const uint8_t seq = ROSCANConstants::Control::seq(msg.can_id);
    ROS_INFO("registering topic for node id %d", nodeID);

    // Key is the static bits of the header
    const uint32_t key = msg.can_id & ~(ROSCANConstants::Common::bitmask_seq | ROSCANConstants::Control::bitmask_seq);

    if (seq == 0) {
        topic_register_buffers.reset(key, len);
    }

    topic_register_buffers.append(key, msg.data, msg.can_dlc);

    #ifdef DEBUG
    const auto& can_buf = topic_register_buffers.get(key);
    char str[1000] = {0};
    sprintf(str, "buffer contents for key %u:", key);
    for (const auto b : can_buf) {
        sprintf(str, "%s %02x", str, b);
    }
    ROS_INFO("%s", str);
    #endif

    if (topic_register_buffers.ready(key)) {
        const auto& can_buf = topic_register_buffers.get(key);
        const auto extracted = extractTopic(can_buf);
        const auto& topic = extracted.first;
        const auto& topic_type = extracted.second;
        const auto node = RosCanNodeManager::instance().getNode(nodeID);
        const auto topicID = mode == ROSCANConstants::Control::SUBSCRIBE_TOPIC ? node->registerSubscriber(topic, topic_type) : node->advertiseTopic(topic, topic_type);

        if (topicID < 0) {
            ROS_INFO("topic register failed");
        } else {
            ROS_INFO("registered topic \"%s\" of type \"%s\" assigned to topic id %d of node id %d", topic.c_str(), topic_type.c_str(), topicID, nodeID);
            can_frame response = msg;
            ROSCANConstants::Control::step_insert(response.can_id, 1);
            ROS_INFO("sending header %x", response.can_id);
            response.can_dlc = 4;
            response.data[0] = topicID;
            CANHelpers::send_frame(response);
        }
    }
}

void CANMsgRouter::resetAllNodes() {
    ROS_INFO("Reseting All Nodes");
    can_frame frame;
    frame.can_dlc = 0;
    canid_t header = 0;
    header |= CAN_EFF_FLAG;
    header |= (1 << ROSCANConstants::Common::bitshift_mode);
    // reset should be high priority to break through any noise
    header |= (0x2 << ROSCANConstants::Common::bitshift_priority);
    header |= (ROSCANConstants::Common::CONTROL << ROSCANConstants::Common::bitshift_func);
    header |= (0 << ROSCANConstants::Common::bitshift_seq);
    header |= (ROSCANConstants::Control::CHANNEL_CONTROL << ROSCANConstants::Control::bitshift_mode);
    header |= CAN_EFF_FLAG;
    frame.can_id = header;
    CANHelpers::send_frame(frame);
    ROS_INFO("Reset Complete");
}
