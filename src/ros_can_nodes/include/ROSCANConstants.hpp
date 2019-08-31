/*
 * Date Started: 20/10/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the BSD License. Copyright BLUEsat UNSW, 2017
 */
#ifndef ROSCANCONSTANTS_H
#define ROSCANCONSTANTS_H

#include <cstdint>

namespace ROSCANConstants {

    /* Generic Value Extractor */
    constexpr auto extractor = [](const uint32_t header, const uint32_t bitshift, const uint32_t bitmask) -> uint8_t {
        return (header & bitmask) >> bitshift;
    };
    
    /** Generic value inserter **/
    constexpr auto inserter = [](uint32_t & frame_header, const uint32_t value, const uint32_t bitshift, const uint32_t bitmask) -> uint32_t {
        frame_header = (frame_header & ~bitmask) | ((value << bitshift) & bitmask);
    };

    namespace Common {

        /* --------------------------------------   */
        /*              Common Msg fields           */
        /* --------------------------------------   */

        /* Bitshifting for common msg fields */
        constexpr uint32_t bitshift_mode = 0;
        constexpr uint32_t bitshift_priority = 1;
        constexpr uint32_t bitshift_func = 3;
        constexpr uint32_t bitshift_seq = 5;

        /* Bitmasking for common msg fields */
        constexpr uint32_t bitmask_mode = (0x1 << bitshift_mode);
        constexpr uint32_t bitmask_priority = (0x3 << bitshift_priority);
        constexpr uint32_t bitmask_func = (0x3 << bitshift_func);
        constexpr uint32_t bitmask_seq = (0x7 << bitshift_seq);

        /* Enumeration for Functions */
        constexpr uint32_t ROS_TOPIC = 0;
        constexpr uint32_t ROS_SERVICE = 1;
        constexpr uint32_t CONTROL = 2;
        constexpr uint32_t RESERVED = 3;

        /* Value extractors */
        constexpr auto mode = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_mode, bitmask_mode); };
        constexpr auto priority = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_priority, bitmask_priority); };
        constexpr auto func = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_func, bitmask_func); };
        constexpr auto seq = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_seq, bitmask_seq); };
        constexpr auto mode_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_mode, bitmask_mode); };
        constexpr auto priority_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_priority, bitmask_priority); };
        constexpr auto func_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_func, bitmask_func); };
        constexpr auto seq_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_seq, bitmask_seq); };
    }

    namespace ROSTopic {

        /* --------------------------------------   */
        /*            ROSTopic Msg fields           */
        /* --------------------------------------   */

        /* Bitshifting for rostopic msg fields */
        constexpr uint32_t bitshift_msg_num = 8;
        constexpr uint32_t bitshift_topic_id = 10;
        constexpr uint32_t bitshift_len = 17;
        constexpr uint32_t bitshift_nid = 25;

        /* Bitmasking for ROS Topic Messages */
        constexpr uint32_t bitmask_msg_num = (0x3 << bitshift_msg_num);
        constexpr uint32_t bitmask_topic_id = (0x7F << bitshift_topic_id);
        constexpr uint32_t bitmask_len = (0xFF << bitshift_len);
        constexpr uint32_t bitmask_nid = (0xF << bitshift_nid);

        /* Value extractors */
        constexpr auto msg_num = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_msg_num, bitmask_msg_num); };
        constexpr auto topic_id = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_topic_id, bitmask_topic_id); };
        constexpr auto len = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_len, bitmask_len); };
        constexpr auto nid = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_nid, bitmask_nid); };
        constexpr auto msg_num_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_msg_num, bitmask_msg_num); };
        constexpr auto topic_id_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_topic_id, bitmask_topic_id); };
        constexpr auto len_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_len, bitmask_len); };
        constexpr auto nid_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_nid, bitmask_nid); };
    }

    namespace Control {

        /* --------------------------------------   */
        /*            Control Msg fields            */
        /* --------------------------------------   */

        /* Bitshifting for Control Msg fields */
        constexpr uint32_t bitshift_mode = 8;

        /* Bitmasking for Control Messages */
        constexpr uint32_t bitmask_mode = (0xF << bitshift_mode);

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
        constexpr uint32_t CHANNEL_CONTROL = 9;
        constexpr uint32_t EXTENDED = 10;

        /* Bitshifting for Control mode 0 */
        constexpr uint32_t bitshift_mode0_step = 12;
        constexpr uint32_t bitshift_mode0_hash = 13;

        /* Bitmasking for  Control mode 0 */
        constexpr uint32_t bitmask_mode0_step = (0x1 << bitshift_mode0_step);
        constexpr uint32_t bitmask_mode0_hash = (0xFF << bitshift_mode0_hash);

        /* Bitshifting for Control modes 1/2/3/4/5 */
        constexpr uint32_t bitshift_nid = 12;

        /* Bitmasking for Control modes 1/2/3/4/5 */
        constexpr uint32_t bitmask_nid = (0xF << bitshift_nid);

        /* Bitshifting for Control modes 2/4 */
        constexpr uint32_t bitshift_step = 16;
        constexpr uint32_t bitshift_hash = 17;
        constexpr uint32_t bitshift_seq = 20;
        constexpr uint32_t bitshift_len = 24;

        /* Bitmasking for Control modes 2/4 */
        constexpr uint32_t bitmask_step = (0x1 << bitshift_step);
        constexpr uint32_t bitmask_hash = (0x7 << bitshift_hash);
        constexpr uint32_t bitmask_seq = (0xF << bitshift_seq);
        constexpr uint32_t bitmask_len = (0xF << bitshift_len);

        /* Bitshifting for Control modes 3/5 */
        constexpr uint32_t bitshift_topic_id = 7;

        /* Bitmasking for Control modes 3/5 */
        constexpr uint32_t bitmask_topic_id = (0x3F << bitshift_topic_id);

        /* Bitshifting for Control mode 9 */
        constexpr uint32_t bitshift_mode9_sub_mode = 12;
        constexpr uint32_t bitshift_mode9_step = 16;

        /* Bitmasking for  Control mode 0 */
        constexpr uint32_t bitmask_mode9_sub_mode = (0xF << bitshift_mode9_sub_mode);
        constexpr uint32_t bitmask_mode9_step = (0x1 << bitshift_mode9_step);
        /* Value extractors */
        constexpr auto mode = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_mode, bitmask_mode); };
        constexpr auto mode0_step = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_mode0_step, bitmask_mode0_step); };
        constexpr auto mode0_hash = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_mode0_hash, bitmask_mode0_hash); };
        constexpr auto nid = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_nid, bitmask_nid); };
        constexpr auto step = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_step, bitmask_step); };
        constexpr auto hash = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_hash, bitmask_hash); };
        constexpr auto seq = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_seq, bitmask_seq); };
        constexpr auto len = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_len, bitmask_len); };
        constexpr auto topic_id = [](const auto header) { return ROSCANConstants::extractor(header, bitshift_topic_id, bitmask_topic_id); };
        constexpr auto mode_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_mode, bitmask_mode); };
        constexpr auto mode0_step_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_mode0_step, bitmask_mode0_step); };
        constexpr auto mode0_hash_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_mode0_hash, bitmask_mode0_hash); };
        constexpr auto nid_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_nid, bitmask_nid); };
        constexpr auto step_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_step, bitmask_step); };
        constexpr auto hash_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_hash, bitmask_hash); };
        constexpr auto seq_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_seq, bitmask_seq); };
        constexpr auto len_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_len, bitmask_len); };
        constexpr auto topic_id_insert = [](uint32_t & header, const uint32_t value) { ROSCANConstants::inserter(header, value, bitshift_topic_id, bitmask_topic_id); };
    }
}

#endif // ROSCANCONSTANTS_H
