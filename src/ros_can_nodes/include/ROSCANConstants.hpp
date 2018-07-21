/*
 * Date Started: 20/10/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */
#ifndef ROSCANCONSTANTS_H
#define ROSCANCONSTANTS_H

#include <cstdint>

namespace ROSCANConstants {

    /* --------------------------------------   */
    /*              Common Msg fields           */
    /* --------------------------------------   */

    /* Bitshifting for common msg fields */
    constexpr uint32_t bitshift_ros_msg = 0;
    constexpr uint32_t bitshift_priority = 1;
    constexpr uint32_t bitshift_func = 3;
    constexpr uint32_t bitshift_seq = 5;

    /* Bitmasking for common msg fields */
    constexpr uint32_t bitmask_ros_msg = (0x1 << bitshift_ros_msg);
    constexpr uint32_t bitmask_priority = (0x3 << bitshift_priority);
    constexpr uint32_t bitmask_func = (0x3 << bitshift_func);
    constexpr uint32_t bitmask_seq = (0x3 << bitshift_seq);

    constexpr uint32_t ROS_TOPIC = 0;
    constexpr uint32_t ROS_SERVICE = 1;
    constexpr uint32_t CONTROL = 2;
    constexpr uint32_t RESERVED = 3;

    /* --------------------------------------   */
    /*            ROSTopic Msg fields           */
    /* --------------------------------------   */

    /* Bitshifting for rostopic msg fields */
    constexpr uint32_t bitshift_msg_num = 8;
    constexpr uint32_t bitshift_topic_id = 10;
    constexpr uint32_t bitshift_len = 16;
    constexpr uint32_t bitshift_nid = 25;

    /* Bitmasking for ROS Topic Messages */
    constexpr uint32_t bitmask_msg_num = (0x3 << bitshift_msg_num);
    constexpr uint32_t bitmask_topic_id = (0x7F << bitshift_topic_id);
    constexpr uint32_t bitmask_len = (0xFF << bitshift_len);
    constexpr uint32_t bitmask_nid = (0xF << bitshift_nid);

    /* --------------------------------------   */
    /*            Control Msg fields          */
    /* --------------------------------------   */

    /* Bitshifting for Control Msg fields */
    constexpr uint32_t bitshift_mode = 8;
    constexpr uint32_t bitshift_mode_specific = 12;

    /* Bitmasking for Control Messages */
    constexpr uint32_t bitmask_mode = (0xF << bitshift_mode);
    constexpr uint32_t bitmask_mode_specific = (0x3FFFF << bitshift_mode_specific);

    /* Enumeration for Control Message Modes */
    constexpr uint32_t REGISTER_NODE = 0;
    constexpr uint32_t DEREGISTER_NODE = 1;
    constexpr uint32_t SUBSCRIBE_TOPIC = 2;
    constexpr uint32_t UNREGISTER_TOPIC = 3;
    constexpr uint32_t ADVERTISE_TOPIC = 4;
    constexpr uint32_t UNREGISTER_PUBLISHER = 5;
    constexpr uint32_t ADVERTISE_SERVICE = 6;
    constexpr uint32_t UNREGISTER_SERVICE = 7;
    constexpr uint32_t PARAMETERS = 8;
    constexpr uint32_t HEARTBEAT = 9;
    constexpr uint32_t EXTENDED = 10;
}
#endif // ROSCANCONSTANTS_H
