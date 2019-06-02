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
#include <vector>
#include <cstdint>
#include <iostream>
#include <ros/console.h>

TopicBuffers& TopicBuffers::instance() {
    static TopicBuffers instance;
    return instance;
}

void TopicBuffers::reset(const short key, uint8_t can_frames) {
    auto& topic = topic_buffers[key];
    ROS_INFO("topic buffers: resetting for key %d", key);
    topic.first.clear();
    topic.second.first = can_frames;
    topic.second.second = 0;
}

bool TopicBuffers::append(const short key, const uint8_t data[CAN_MAX_DLEN], const int dlc) {
    auto& topic = topic_buffers[key];
    ROS_INFO("topic buffers: appending %d bytes to key %d", dlc, key);
    topic.first.insert(topic.first.end(), data, data + dlc);
    ++topic.second.second;
    return topic.second.first == topic.second.second;
}

std::vector<uint8_t>& TopicBuffers::get(const short key) {
    return topic_buffers[key].first;
}
