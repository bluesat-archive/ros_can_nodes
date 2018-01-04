/*
 * Date Started: 20/10/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */

#include <unordered_map>

#define MAX_CAN_MSGS 32
#define TOPIC_BUFFER_SIZE (CAN_MAX_DLEN * MAX_CAN_MSGS)
typedef uint8_t[TOPIC_BUFFER_SIZE] buffer;

class TopicBuffers{

    std::unordered_map<int, buffer> topic_buffers;

    static void initBuffers(void);

    static void processData(int key, uint8_t* data, int d_len, bool last_msg);

    static void appendData(uint8_t* data);

    // TODO: Possible Custom Hash function for unordered_map
    // TODO: Possible Custom Equality function for unordered_map
}
