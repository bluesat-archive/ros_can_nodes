/*
 * Date Started: 20/10/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */

#include "ROSCANConstants.hpp"
#include "TopicBuffers.hpp"

TopicBuffers& TopicBuffers::instance() {
    static TopicBuffers instance;
    return instance;
}

void TopicBuffers::initBuffers() {

}

void TopicBuffers::appendData(const short key, const uint8_t data) {
    //TODO: append data
    topic_buffers[key][0] = data;
}

void TopicBuffers::processData(const short key, const uint8_t *const data, const int d_len, const bool last_msg) {
    appendData(key, *data);

    if (last_msg) {
        // TODO: Convert to flattype and publish to ros network
    }
}
