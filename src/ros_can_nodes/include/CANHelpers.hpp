/*
 * Date Started: 3/12/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */

#ifndef CANHELPERS_HPP
#define CANHELPERS_HPP

#include <linux/can.h>
#include <string>

namespace CANHelpers {

    /**
     * Opens a CAN port on the given port string
     * Returns -1 if there was an error, 0 otherwise
     */
    int open_port(const std::string& port);

    /**
     * Sends out a CAN frame
     * A valid CAN port must be open
     * Returns -1 if there was an error, 0 otherwise
     */
    int send_frame(const can_frame& frame);

    /**
     * Reads in a CAN frame
     * A valid CAN port must be open
     * Returns -1 if there was an error, 0 otherwise
     */
    int read_frame(can_frame& frame);

    /**
     * Closes the CAN port
     */
    void close_port();

} // namespace CANHelpers

#endif // CANHELPERS_HPP
