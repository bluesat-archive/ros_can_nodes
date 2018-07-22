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
    constexpr uint32_t bitshift_ros_msg = 28;
    constexpr uint32_t bitshift_priority = 26;
    constexpr uint32_t bitshift_func = 24;
    constexpr uint32_t bitshift_seq = 21;

    /* Bitmasking for common msg fields */
    constexpr uint32_t bitmask_ros_msg = (0x1 << bitshift_ros_msg);
    constexpr uint32_t bitmask_priority = (0x3 << bitshift_priority);
    constexpr uint32_t bitmask_func = (0x3 << bitshift_func);
    constexpr uint32_t bitmask_seq = (0x7 << bitshift_seq);

    constexpr uint32_t ROS_TOPIC = 0;
    constexpr uint32_t ROS_SERVICE = 1;
    constexpr uint32_t CONTROL = 2;
    constexpr uint32_t RESERVED = 3;

    /* --------------------------------------   */
    /*            ROSTopic Msg fields           */
    /* --------------------------------------   */

    /* Bitshifting for rostopic msg fields */
    constexpr uint32_t bitshift_msg_num = 19;
    constexpr uint32_t bitshift_topic_id = 13;
    constexpr uint32_t bitshift_len = 4;
    constexpr uint32_t bitshift_nid = 0;

    /* Bitmasking for ROS Topic Messages */
    constexpr uint32_t bitmask_msg_num = (0x3 << bitshift_msg_num);
    constexpr uint32_t bitmask_topic_id = (0x3F << bitshift_topic_id);
    constexpr uint32_t bitmask_len = (0x1FF << bitshift_len);
    constexpr uint32_t bitmask_nid = (0xF << bitshift_nid);

    /* --------------------------------------   */
    /*            Control Msg fields            */
    /* --------------------------------------   */

    /* Bitshifting for Control Msg fields */
    constexpr uint32_t bitshift_mode = 17;
    constexpr uint32_t bitshift_mode_specific = 1;

    /* Bitmasking for Control Messages */
    constexpr uint32_t bitmask_mode = (0xF << bitshift_mode);
    constexpr uint32_t bitmask_mode_specific = (0xFFFF << bitshift_mode_specific);

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

    /* Bitshifting for Control mode 0 */
    constexpr uint32_t bitshift_mode0_step = 16;
    constexpr uint32_t bitshift_mode0_hash = 8;

    /* Bitmasking for  Control mode 0 */
    constexpr uint32_t bitmask_mode0_step = (0x1 << bitshift_mode0_step);
    constexpr uint32_t bitmask_mode0_hash = (0xFF << bitshift_mode0_hash);

    /* Bitshifting for Control modes 1/2/3/4/5 */
    constexpr uint32_t bitshift_nodeid = 13;

    /* Bitmasking for Control modes 1/2/3/4/5 */
    constexpr uint32_t bitmask_nodeid = (0xF << bitshift_nodeid);

    /* Bitshifting for Control modes 2/4 */
    constexpr uint32_t bitshift_step = 12;
    constexpr uint32_t bitshift_hash = 9;
    constexpr uint32_t bitshift_seqnum = 5;
    constexpr uint32_t bitshift_topiclen = 1;

    /* Bitmasking for Control modes 2/4 */
    constexpr uint32_t bitmask_step = (0x1 << bitshift_step);
    constexpr uint32_t bitmask_hash = (0x7 << bitshift_hash);
    constexpr uint32_t bitmask_seqnum = (0xF << bitshift_seqnum);
    constexpr uint32_t bitmask_topiclen = (0xF << bitshift_topiclen);

    /* Bitshifting for Control modes 3/5 */
    constexpr uint32_t bitshift_topicid = 7;

    /* Bitmasking for Control modes 3/5 */
    constexpr uint32_t bitmask_topicid = (0x3F << bitshift_topicid);
}

#endif // ROSCANCONSTANTS_H
