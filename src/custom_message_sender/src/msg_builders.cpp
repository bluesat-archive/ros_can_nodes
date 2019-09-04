#include <ROSCANConstants.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <cstdint>
#include <linux/can.h>
#include "msg_builders.hpp"

can_frame common_msg(const uint8_t mode_bit, const uint8_t priority) {
    can_frame msg;
    msg.can_id = CAN_EFF_FLAG;
    ROSCANConstants::Common::mode_insert(msg.can_id, mode_bit);
    ROSCANConstants::Common::priority_insert(msg.can_id, priority);
    msg.can_dlc = 0;
    return msg;
}

can_frame build_register_node_msg(const std::string& node_name) {
    if (node_name.size() > 8) {
        throw new std::runtime_error("Node names greater than 8 bytes is not yet supported");
    }
    auto msg = common_msg();
    ROSCANConstants::Common::func_insert(msg.can_id, ROSCANConstants::Common::CONTROL);
    ROSCANConstants::Control::mode_insert(msg.can_id, ROSCANConstants::Control::REGISTER_NODE);
    msg.can_dlc = node_name.size();
    std::copy(node_name.cbegin(), node_name.cend(), msg.data);
    return msg;
}

std::vector<can_frame> build_topic_control_msgs(const uint32_t mode, const uint32_t node_id, const uint32_t hash, const std::string& topic_name, const std::string& msg_type) {
    std::vector<uint8_t> data;
    std::copy(topic_name.cbegin(), topic_name.cend(), std::back_inserter(data));
    data.push_back(0);
    std::copy(msg_type.cbegin(), msg_type.cend(), std::back_inserter(data));
    data.push_back(0);

    const auto msgs_count = data.size() / 8 + (data.size() % 8 != 0);

    auto msg = common_msg();
    ROSCANConstants::Common::func_insert(msg.can_id, ROSCANConstants::Common::CONTROL);
    ROSCANConstants::Control::mode_insert(msg.can_id, mode);
    ROSCANConstants::Control::nid_insert(msg.can_id, node_id);
    ROSCANConstants::Control::hash_insert(msg.can_id, hash);
    ROSCANConstants::Control::len_insert(msg.can_id, msgs_count);

    std::vector<can_frame> frames;
    for (uint32_t i = 0;i < msgs_count;++i) {
        const auto offset = i * 8;
        auto frame_i = msg;
        ROSCANConstants::Control::seq_insert(frame_i.can_id, i);
        frame_i.can_dlc = std::min(data.size(), static_cast<size_t>(offset + 8)) - offset;
        std::copy(data.cbegin() + offset, data.cbegin() + offset + frame_i.can_dlc, frame_i.data);
        frames.push_back(frame_i);
    }
    return frames;
}
