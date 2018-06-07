/*
 * Date Started: 3/12/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */

class CANHelpers{

    static int open_can_port(const char *port);

    static int send_can_port(struct can_frame *frame);

    static int read_can_port(struct can_frame *frame);

    static void close_can_port();
}
