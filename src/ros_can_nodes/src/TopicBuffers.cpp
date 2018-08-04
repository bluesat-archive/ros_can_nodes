/*
 * Date Started: 20/10/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */

#include "TopicBuffers.hpp"
#include <algorithm>

TopicBuffers& TopicBuffers::instance() {
    static TopicBuffers instance;
    return instance;
}

void TopicBuffers::initBuffers() {}

void TopicBuffers::processData(const short key, const uint8_t data[CAN_MAX_DLEN], const int dlc, const bool last_msg, double& value) {
    // get buffer by key
    auto& buffer = topic_buffers[key];

    // reserve max buffer size if buffer was never used before
    if (buffer.capacity() < TOPIC_BUFFER_SIZE) {
        buffer.reserve(TOPIC_BUFFER_SIZE);
    }
    // append buffer
    buffer.insert(buffer.end(), data, data + dlc);

    if (last_msg) {
        // reverse endianness
        std::reverse(buffer.begin(), buffer.end());

        // convert buffer to data
        value = *(double *)buffer.data();

        // recycle the buffer
        buffer.clear();
    }
}
