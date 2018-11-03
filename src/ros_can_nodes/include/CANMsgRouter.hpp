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

class CANMsgRouter {

    public:

        /**
         * Initialises
         */
        static void init();

        static void run();

        /**
         * Sets up some dummy publisher messages
         */
        static void publisherTest();

        /**
         * Sets up some dummy subscribers
         */
        static void subscriberTest();

        static void processCANMsg(const can_frame& msg);

        static void routeControlMsg(const can_frame& msg);

        static void routePublishMsg(const can_frame& msg);

        static void extractTopic(const can_frame& first, std::string& topic, std::string& topic_type);

    private:

};

#endif // CANMSGROUTER_H
