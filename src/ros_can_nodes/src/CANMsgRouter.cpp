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
#include "CANMsgRouter.hpp"
#include "ROSCANConstants.hpp"
#include "TopicBuffers.hpp"
#include "CANHelpers.hpp"
#include "ros_node_lib/network.h"
#include "RosCanNode.hpp"
#include "RosCanNodeManager.hpp"


int main(int argc, char **argv) {
    roscan::network::init();

    CANMsgRouter::init();

    CANMsgRouter::subscriberTest();

    while (1) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    //CANMsgRouter::run();
}

void CANMsgRouter::init() {
    // TODO: either fail on bad open_port OR have reconnect policy
    int err = CANHelpers::open_can_port("can0");

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
        int numbytes = CANHelpers::read_can_port(&can_msg);

        if (numbytes >= 0){
            // Pass to processCANMsg
            CANMsgRouter::processCANMsg(can_msg);
        }
    }
}

void CANMsgRouter::subscriberTest() {
    std::string name = "testNode";
    uint8_t nodeId = RosCanNodeManager::instance().registerNode(name, 0);

    roscan::RosCanNodePtr node = RosCanNodeManager::instance().getNode(nodeId);

    node->registerSubscriber("/front_left_wheel_axel_controller/command", "blah");
    node->registerSubscriber("/back_left_wheel_axel_controller/command", "blah");
    node->registerSubscriber("/front_left_swerve_controller/command", "blah");
    node->registerSubscriber("/back_left_swerve_controller/command", "blah");

}

void CANMsgRouter::processCANMsg(can_frame msg) {
    // Check the CAN Msg Header to perform routing
    uint8_t msg_header = msg.can_id & CAN_ERR_MASK;

    if ((msg_header & ROSCANConstants::bitmask_ros_msg) == 0){
            // Out-of-Channel communication handling
            // Currently unimplemented

    } else {
        // Check the function
        uint8_t msg_function = ((msg_header & ROSCANConstants::bitmask_func) >> ROSCANConstants::bitshift_func);

        if (msg_function == ROSCANConstants::ROS_TOPIC) {

            // Start thread to handle a ros message packet
            // Mutex is placed within handler on a per-node basis, so that
            // no 2 topics from a single node can be concurrently handling
            CANMsgRouter::routePublishMsg(msg);


        } else if (msg_function == ROSCANConstants::ROS_SERVICE) {
            // ROS_SERVICE message handling
            // Currently unimplemented

        } else if (msg_function == ROSCANConstants::CONTROL) {
            // Assume control messages are valid and intended for main controller
            // Start a thread to handle control message.
            // TODO: handle shared resource with topicThread (nodeList, topics etc)
            CANMsgRouter::routeControlMsg(msg);

        } else {
            // -- Reserved --
        }
    }
}

// Function to cut the control msg into its components, and then call the
// appropriate Node function.
void CANMsgRouter::routeControlMsg(can_frame msg) {
    uint8_t mode = ((msg.can_id & ROSCANConstants::bitmask_mode) >> ROSCANConstants::bitshift_mode);

    uint8_t mode_info = ((msg.can_id & ROSCANConstants::bitmask_mode_specific) >> ROSCANConstants::bitshift_mode_specific);

    switch (mode) {
        case ROSCANConstants::REGISTER_NODE:
            {
                uint8_t step = mode_info & 0x1;
                uint8_t hashName = (mode_info >> 1) & 0xF;

                std::ostringstream convert;
                for (int a = 0; a < msg.can_dlc; a++) {
                    convert << (int)msg.data[a];
                }

                std::string name = convert.str();

                RosCanNodeManager::instance().registerNode(name, hashName);
            }
        case ROSCANConstants::DEREGISTER_NODE:
            {
                uint8_t nodeID = mode_info & 0xF;
                RosCanNodeManager::instance().deregisterNode(nodeID);
            }
        case ROSCANConstants::SUBSCRIBE_TOPIC:
            {
                uint8_t nodeID = mode_info & 0xF;
                //TODO: get data content for function call
                //roscan::RosCanNode::getNode(nodeID)->registerSubscriber(topic, topic_type);
            }
        case ROSCANConstants::UNREGISTER_TOPIC:
            {
                uint8_t nodeID = mode_info & 0xF;
                uint8_t topicID = (mode_info >> 4) & 0x3F;
                RosCanNodeManager::instance().getNode(nodeID)->unregisterSubscriber(topicID);
            }
        case ROSCANConstants::ADVERTISE_TOPIC:
            {
                uint8_t nodeID = mode_info & 0xF;
                //TODO: get data content for function call
                //roscan::RosCanNode::getNode(nodeID)->advertiseTopic(std::string topic, std::string topic_type);
            }
        case ROSCANConstants::UNREGISTER_PUBLISHER:
            {
                uint8_t nodeID = mode_info & 0xF;
                uint8_t topicID = (mode_info >> 4) & 0x3F;
                 RosCanNodeManager::instance().getNode(nodeID)->unregisterPublisher(topicID);
            }
        case ROSCANConstants::ADVERTISE_SERVICE:
            {
                uint8_t nodeID = mode_info & 0xF;
                //roscan::RosCanNode::getNode(nodeID)->advertiseService(std::string service);
            }
        case ROSCANConstants::UNREGISTER_SERVICE:
            {
                uint8_t nodeID = mode_info & 0xF;
                //roscan::RosCanNode::getNode(nodeID)->unregisterService(std::string service);
            }
        case ROSCANConstants::PARAMETERS:
            {
                //TODO: later
            }
        case ROSCANConstants::HEARTBEAT:
            {
                uint8_t nodeID = mode_info & 0xF;
                //roscan::RosCanNode::getNode(nodeID)->heartbeat(void);
            }
        case ROSCANConstants::EXTENDED:
            {
                //TODO: later
            }
        default:
            {
            //NOTE: not a valid mode
            }
    }
}

void CANMsgRouter::routePublishMsg(can_frame msg) {

    uint8_t msg_header = msg.can_id & CAN_ERR_MASK;

    // Check if we have reached the last expected msg
    bool last_msg = (msg.can_dlc < 8) ? false : true;

    // Grab attributes needed for accessing the buffer
    uint8_t topic = (msg_header >> ROSCANConstants::bitshift_topic_id) & ROSCANConstants::bitmask_topic_id;
    uint8_t nid = (msg_header >> ROSCANConstants::bitshift_nid) & ROSCANConstants::bitmask_nid;

    uint8_t seq = (msg_header >> ROSCANConstants::bitshift_seq) & ROSCANConstants::bitmask_seq;

    // Special case for messages that are made up of 7 or more can packets
    if (seq == 7) {
        seq = (msg_header >> ROSCANConstants::bitshift_len) & ROSCANConstants::bitmask_len;
    }

    // Key will be concatenation of topicid, and nid
    short key = (topic << 5) | nid;

    TopicBuffers::instance().processData(key, msg.data, seq, last_msg);
}
