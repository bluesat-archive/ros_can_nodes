/*
 * Date Started: 29/09/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */

#ifndef CANMSGROUTER_H
#define CANMSGROUTER_H

#include <linux/can.h>
#include <cstdint>

class CANMsgRouter{

    public:

    private:

        void init();

        void run();

        bool processCANMsg(can_frame msg);

        void routeControlMsg(uint32_t identifier, uint8_t *data);

        void routePublishMsg(uint32_t identifier, uint8_t *data);
};

#endif // CANMSGROUTER_H
