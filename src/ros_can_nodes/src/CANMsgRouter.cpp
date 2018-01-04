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

int main(int argc, char **argv){
    ros::init(argc, argv, "can_ros_decoder");

    CANMsgRoute::init();

    CANMsgRouter::run();
}

static void CANMsgRouter::init(){
    // TODO: either fail on bad open_port OR have reconnect policy
    CANHelpers::open_can_port('can0');

    TopicBuffers::initBuffers();
}

static void CANMsgRouter::run(){
    struct can_frame can_msg;

    while(1){
        // Check for Messages
        can_msg = read_can_port();

        if (can_msg != NULL){
            // Pass to processCANMsg
            CANMsgRouter::processCANMsg(can_msg);
        }
    }
}

static void CANMsgRouter::processCANMsg(can_frame msg){
    // Check the CAN Msg Header to perform routing
    uint msg_header = msg.can_id & CAN_ERR_MASK;

    if ((msg_header & bitmask_ros_msg) == 0){
            // TODO: Out-of-Channel communication handling
        else {
            // Check the function
            uint msg_function = ((msg_header & bitmask_func) >> bitshift_func);

            if(msg_function == ROSCANConstants::msg_func.ROS_TOPIC) {
                CANMsgRouter::routePublishMsg(msg);
            } else if (msg_function == ROSCANConstants::msg_func.ROS_SERVICE)) {
                // TODO: Implement ROS_SERVICE message handling
            } else if (msg_function == ROSCANConstants::msg_func.CONTROL)) {
                CANMsgRouter::routeControlMsg(msg);
            } else {
                // TODO -- Reserved --
            }
        }
    }
}

static void CANMsgRouter::routeControlMsg(can_frame msg){
    //TODO: link with Tristan's code
}

static void CANMsgRouter::routePublishMsg(can_frame msg){

    bool last_msg = (msg.can_dlc < 8) ? FALSE : TRUE;

    int key = 0;

    TopicBuffers::processData( key,data, msg.can_dlc, last_msg);
}
