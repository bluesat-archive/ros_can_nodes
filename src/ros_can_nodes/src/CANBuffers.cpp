/*
 * Date Started: 20/10/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose: Byte buffer manager for CAN message frame data. Each buffer can be accessed
 *          and modified using a unique key
 * This code is released under the BSD License. Copyright BLUEsat UNSW, 2017
 */

#include "CANBuffers.hpp"
#include <vector>
#include <cstdint>
#include <iostream>
#include <ros/console.h>
#include <linux/can.h>

void CANBuffers::reset(const uint32_t key, const uint8_t expected_frames) {
    auto& buffer = buffers[key];
    ROS_INFO("CAN buffers: resetting for key %u", key);
    buffer.buf.clear();
    buffer.expected_frames = expected_frames;
    buffer.received_frames = 0;
}

void CANBuffers::append(const uint32_t key, const uint8_t data[CAN_MAX_DLEN], const uint8_t data_len) {
    auto& buffer = buffers.at(key);
    ROS_INFO("CAN buffers: appending %u bytes to key %u", data_len, key);
    buffer.buf.insert(buffer.buf.cend(), data, data + data_len);
    ++buffer.received_frames;
}

bool CANBuffers::ready(const uint32_t key) {
    if (buffers.count(key) == 0) {
        return false;
    }
    auto& buffer = buffers.at(key);
    return buffer.expected_frames == buffer.received_frames;
}

const std::vector<uint8_t>& CANBuffers::get(const uint32_t key) {
    return buffers.at(key).buf;
}
