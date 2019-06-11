/**
 * Date Started:
 * Original Author: Yiwei Han
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose: Helper module for manipulating raw ROS message data
 * This code is released under the MIT License. Copyright BLUEsat UNSW, 2019
 */

#ifndef INTROSPECTIONHELPERS_HPP
#define INTROSPECTIONHELPERS_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <ros_type_introspection/utils/shape_shifter.hpp>

namespace IntrospectionHelpers {
    /**
     * Register message type for ros message type introspection
     * Message registration is thread safe
     */
    void register_message(const std::string& datatype, const std::string& definition);

    /**
     * Print all registered message types and fields structures
     */
    void print_registered();

    /**
     * Modifies a ROS message data buffer for communication on the CAN bus
     * datatype must be a registered message type
     */
    std::vector<uint8_t> to_can_buf(const std::string& datatype, const uint8_t *const data, const uint32_t size);

    /**
     * Modifies a CAN message data buffer for communication in ROS
     * datatype must be a registered message type
     */
    std::vector<uint8_t> to_ros_buf(const std::string& datatype, const uint8_t *const data, const uint32_t size);
}

#endif // INTROSPECTIONHELPERS_HPP