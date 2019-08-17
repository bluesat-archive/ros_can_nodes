#include <iostream>
#include <string>
#include <algorithm>
#include <iterator>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <ROSCANConstants.hpp>
#include "can.hpp"
#include "msg_builders.hpp"

static void send(const can_frame& frame) {
    if (send_can_frame(frame) < 0) {
        throw new std::runtime_error("Frame send failed");
    }
}

static std::vector<can_frame> zip(const std::vector<std::vector<can_frame>>& sequences) {
    std::vector<uint32_t> sequence_lens;
    std::transform(sequences.cbegin(), sequences.cend(), std::back_inserter(sequence_lens), [](const auto& s){ return s.size(); });
    const auto max_len = *std::max_element(sequence_lens.cbegin(), sequence_lens.cend());

    std::vector<can_frame> flat;
    for (uint32_t i = 0;i < max_len;++i) {
        for (uint32_t s = 0;s < sequences.size();++s) {
            if (i < sequences[s].size()) {
                flat.push_back(sequences[s][i]);
            }
        }
    }
    return flat;
}

int main(int argc, char *argv[]) {
    if (open_can_port("vcan0") < 0) {
        throw new std::runtime_error("Could not open port");
    }
    send(build_register_node_msg("mynode"));

    const auto msgs0 = build_topic_control_msgs(ROSCANConstants::Control::SUBSCRIBE_TOPIC, 0, 0, "/00000000000", "std_msgs/String");
    const auto msgs1 = build_topic_control_msgs(ROSCANConstants::Control::ADVERTISE_TOPIC, 0, 1, "/11111111111", "std_msgs/String");
    const auto msgs2 = build_topic_control_msgs(ROSCANConstants::Control::SUBSCRIBE_TOPIC, 0, 2, "/22222222222", "std_msgs/String");
    const auto msgs3 = build_topic_control_msgs(ROSCANConstants::Control::ADVERTISE_TOPIC, 0, 3, "/33333333333", "std_msgs/String");
    const auto msgs4 = build_topic_control_msgs(ROSCANConstants::Control::SUBSCRIBE_TOPIC, 0, 4, "/44444444444", "std_msgs/String");
    const auto msgs5 = build_topic_control_msgs(ROSCANConstants::Control::ADVERTISE_TOPIC, 0, 5, "/55555555555", "std_msgs/String");
    const auto sequences = std::vector<std::vector<can_frame>>{
        msgs0,
        msgs1,
        msgs2,
        msgs3,
        msgs4,
        msgs5
    };
    const auto zipped = zip(sequences);
    for (const auto msg : zipped) {
        send(msg);
    }
    return 0;
}
