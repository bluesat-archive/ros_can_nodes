#include <cstdint>
#include <linux/can.h>
#include <string>
#include <vector>

can_frame common_msg(const uint8_t mode_bit = 1, const uint8_t priority = 0);
can_frame build_register_node_msg(const std::string& node_name);
std::vector<can_frame> build_topic_control_msgs(const uint32_t mode, const uint32_t node_id, const uint32_t hash, const std::string& topic_name, const std::string& msg_type);