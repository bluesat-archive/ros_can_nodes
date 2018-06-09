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

namespace ROSCANConstants{

    /* --------------------------------------   */
    /*              Common Msg fields           */
    /* --------------------------------------   */

    /* Bitshifting for common msg fields */
    static const uint bitshift_ros_msg = 0;
    static const uint bitshift_priority = 1;
    static const uint bitshift_func = 3;
    static const uint bitshift_seq = 5;

    /* Bitmasking for common msg fields */
    static const uint bitmask_ros_msg = (0x1 << bitshift_ros_msg);
    static const uint bitmask_priority = (0x3 << bitshift_priority);
    static const uint bitmask_func = (0x3 << bitshift_func);
    static const uint bitmask_seq = (0x3 << bitshift_seq);

    static const enum msg_func{
        ROS_TOPIC,
        ROS_SERVICE,
        CONTROL,
        RESERVED
    };

    /* --------------------------------------   */
    /*            ROSTopic Msg fields           */
    /* --------------------------------------   */

    /* Bitshifting for rostopic msg fields */
    static const uint bitshift_msg_num = 8;
    static const uint bitshift_topic_id = 10;
    static const uint bitshift_len = 17;
    static const uint bitshift_nid = 25;

    /* Bitmasking for ROS Topic Messages */
    static const uint bitmask_msg_num = (0x3 << bitshift_msg_num);
    static const uint bitmask_topic_id = (0x7F << bitshift_topic_id);
    static const uint bitmask_len = (0xFF << bitshift_len);
    static const uint bitmask_nid = (0xF << bitshift_nid);

    /* --------------------------------------   */
    /*            Control Msg fields          */
    /* --------------------------------------   */

    /* Bitshifting for Control Msg fields */
    static const uint bitshift_mode = 8;
    static const uint bitshift_mode_specific = 12;

    /* Bitmasking for Control Messages */
    static const uint bitmask_mode = (0xF << bitshift_mode);
    static const uint bitmask_mode_specific = (0x3FFFF << bitshift_mode_specific)

    /* Enumeration for Control Message Modes */
    static const enum control_modes{
        REGISTER_NODE,
        DEREGISTER_NODE,
        SUBSCRIBE_TOPIC,
        UNREGISTER_TOPIC,
        ADVERTISE_TOPIC,
        UNREGISTER_PUBLISHER,
        ADVERTISE_SERVICE,
        UNREGISTER_SERVICE,
        PARAMETERS,
        HEARTBEAT,
        EXTENDED
    };
}
#endif // ROSCANCONSTANTS_H
