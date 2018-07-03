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
#include "CANMsgRouter.hpp"
#include "ROSCANConstants.hpp"
#include "TopicBuffers.hpp"
#include "CANHelpers.hpp"
#include "RosCanNode.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "can_ros_decoder");

    CANMsgRouter::init();

    CANMsgRouter::run();
}

static void CANMsgRouter::init(){
    // TODO: either fail on bad open_port OR have reconnect policy
    int err = open_can_port("can0");

    if(err){
        throw "Failed to acuire can socket, exiting";
    }

    TopicBuffers::initBuffers();
}

static void CANMsgRouter::run(){
    struct can_frame can_msg;

    while(1){
        // Check for Messages
        // TODO add reconnection ability
        int numbytes = read_can_port(&can_msg);

        if (numbytes >= 0){
            // Pass to processCANMsg
            CANMsgRouter::processCANMsg(can_msg);
        }
    }
}

static void CANMsgRouter::processCANMsg(can_frame msg){
    // Check the CAN Msg Header to perform routing
    uint8_t msg_header = msg.can_id & CAN_ERR_MASK;

    if ((msg_header & bitmask_ros_msg) == 0){
            // Out-of-Channel communication handling
            // Currently unimplemented

    } else {
        // Check the function
        uint8_t msg_function = ((msg_header & bitmask_func) >> bitshift_func);

        if(msg_function == ROSCANConstants::msg_func.ROS_TOPIC) {

            // Start thread to handle a ros message packet
            // Mutex is placed within handler on a per-node basis, so that
            // no 2 topics from a single node can be concurrently handling
            CANMsgRouter::routePublishMsg(msg);


        } else if (msg_function == ROSCANConstants::msg_func.ROS_SERVICE)) {
            // ROS_SERVICE message handling
            // Currently unimplemented

        } else if (msg_function == ROSCANConstants::msg_func.CONTROL)) {
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
static void CANMsgRouter::routeControlMsg(can_frame msg){
    uint8_t mode = ((msg & bitmask_mode) >> bitshift_mode);

    uint8_t mode_info = ((msg & bitmask_mode_specific) >> bitshift_mode_specific);

    switch(mode){
        case control_modes.REGISTER_NODE:

            uint8_t step = mode_info & 0x1;
            uint8_t nodeHash = (mode_info >> 1) & 0xF);
            std::string name = msg.data;
            RosCanNode::registerNode(name, hashId);

        case control_modes.DEREGISTER_NODE:

            uint8_t nodeID = mode_info & 0xF;
            RosCanNode::getNode(nodeID)->deregisterNode(nodeID);

        case control_modes.SUBSCRIBE_TOPIC:

            uint8_t nodeID = mode_info & 0xF;
            RosCanNode::getNode(nodeID)->registerSubscriber(std::string topic, std::string topic_type);

        case control_modes.UNREGISTER_TOPIC:

            uint8_t nodeID = mode_info & 0xF;
            uint8_t topicID = (mode_info >> 4) & 0x3F;
            RosCanNode::getNode(nodeID)->unregisterSubscriber(topicID);

        case control_modes.ADVERTISE_TOPIC:

            uint8_t nodeID = mode_info & 0xF;
            RosCanNode::getNode(nodeID)->advertiseTopic(std::string topic, std::string topic_type);

        case control_modes.UNREGISTER_PUBLISHER:

            uint8_t nodeID = mode_info & 0xF;
            uint8_t topicID = (mode_info >> 4) & 0x3F;
            RosCanNode::getNode(nodeID)->unregisterPublisher(topicID);

        case control_modes.ADVERTISE_SERVICE:

            uint8_t nodeID = mode_info & 0xF;
            //RosCanNode::getNode(nodeID)->advertiseService(std::string service);

        case control_modes.UNREGISTER_SERVICE:

            uint8_t nodeID = mode_info & 0xF;
            //RosCanNode::getNode(nodeID)->unregisterService(std::string service);

        case control_modes.PARAMETERS:

            //TODO: later

        case control_modes.HEARTBEAT:

            uint8_t nodeID = mode_info & 0xF;
            //RosCanNode::getNode(nodeID)->heartbeat(void);

        case control_modes.EXTENDED:

            //TODO: later

        default:

            //NOTE: not a valid mode

    }
}

static void CANMsgRouter::routePublishMsg(can_frame msg){

    uint8_t msg_header = msg.can_id & CAN_ERR_MASK;

    // Check if we have reached the last expected msg
    bool last_msg = (msg.can_dlc < 8) ? FALSE : TRUE;

    // Grab attributes needed for accessing the buffer
    uint8_t topic = (msg >> bitshift_topic_id) & bitmask_topic_id;
    uint8_t nid = (msg >> bitshift_nid) & bitmask_nid;

    uint8_t seq = (msg >> bitshift_seq) & bitmask_seq;

    // Special case for messages that are made up of 7 or more can packets
    if (seq == 7) {
        seq = (msg >> bitshift_len) & bitmask_len;
    }

    // Key will be concatenation of topicid, and nid
    short key = (topic << 5) | nid;

    TopicBuffers::processData( key, msg.data, seq, last_msg);
}
