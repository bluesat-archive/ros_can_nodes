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
#include <thread>

int main(int argc, char **argv){
    ros::init(argc, argv, "can_ros_decoder");

    CANMsgRoute::init();

    CANMsgRouter::run();
}

static void CANMsgRouter::init(){
    // TODO: either fail on bad open_port OR have reconnect policy
    int err = CANHelpers::open_can_port('can0');

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
    uint msg_header = msg.can_id & CAN_ERR_MASK;

    if ((msg_header & bitmask_ros_msg) == 0){
            // Out-of-Channel communication handling
            // Currently unimplemented

    } else {
        // Check the function
        uint msg_function = ((msg_header & bitmask_func) >> bitshift_func);

        if(msg_function == ROSCANConstants::msg_func.ROS_TOPIC) {

            // Start thread to handle a ros message packet
            // Mutex is placed within handler on a per-node basis, so that
            // no 2 topics from a single node can be concurrently handling
            std::thread topicThread (CANMsgRouter::routePublishMsg, msg);


        } else if (msg_function == ROSCANConstants::msg_func.ROS_SERVICE)) {
            // ROS_SERVICE message handling
            // Currently unimplemented

        } else if (msg_function == ROSCANConstants::msg_func.CONTROL)) {
            // Assume control messages are valid and intended for main controller
            // Start a thread to handle control message.
            // TODO: handle shared resource with topicThread (nodeList, topics etc)
            std::thread controlThread (CANMsgRouter::routeControlMsg, msg);
            controlThread.join();

        } else {
            // -- Reserved --
        }
    }
}

// Function to cut the control msg into its components, and then call the
// appropriate Node function.
static void CANMsgRouter::routeControlMsg(can_frame msg){
    uint mode = ((msg & bitmask_mode) >> bitshift_mode);

    uint mode_info = ((msg & bitmask_mode_specific) >> bitshift_mode_specific);

    switch(mode){
        case control_modes.REGISTER_NODE:

            uint step = mode_info & 0x1;
            uint nodeHash = (mode_info >> 1) & 0xF);
            std::string name = msg.data;
            NodeClass::registerNode(name, hashId);

        case control_modes.DEREGISTER_NODE:

            uint nodeID = mode_info & 0xF;
            NodeClass::getNode(nodeID)->deregisterNode(nodeID);

        case control_modes.SUBSCRIBE_TOPIC:

            uint nodeID = mode_info & 0xF;
            NodeClass::getNode(nodeID)->registerSubscriber(std::string topic, std::string topic_type);

        case control_modes.UNREGISTER_TOPIC:

            uint nodeID = mode_info & 0xF;
            uint topicID = (mode_info >> 4) & 0x3F;
            NodeClass::getNode(nodeID)->unregisterSubscriber(topicID);

        case control_modes.ADVERTISE_TOPIC:

            uint nodeID = mode_info & 0xF;
            NodeClass::getNode(nodeID)->advertiseTopic(std::string topic, std::string topic_type);

        case control_modes.UNREGISTER_PUBLISHER:

            uint nodeID = mode_info & 0xF;
            uint topicID = (mode_info >> 4) & 0x3F;
            NodeClass::getNode(nodeID)->unregisterPublisher(topicID);

        case control_modes.ADVERTISE_SERVICE:

            uint nodeID = mode_info & 0xF;
            //NodeClass::getNode(nodeID)->advertiseService(std::string service);

        case control_modes.UNREGISTER_SERVICE:

            uint nodeID = mode_info & 0xF;
            //NodeClass::getNode(nodeID)->unregisterService(std::string service);

        case control_modes.PARAMETERS:

            //TODO: later

        case control_modes.HEARTBEAT:

            uint nodeID = mode_info & 0xF;
            //NodeClass::getNode(nodeID)->heartbeat(void);

        case control_modes.EXTENDED:

            //TODO: later

        default:

            //NOTE: not a valid mode

    }
}

static void CANMsgRouter::routePublishMsg(can_frame msg){

    // Check if we have reached the last expected msg
    // TODO: this doesn't handle if last msg is 8 in length!
    bool last_msg = (msg.can_dlc < 8) ? FALSE : TRUE;

    int key = 0;

    TopicBuffers::processData( key, data, msg.can_dlc, last_msg);
}
