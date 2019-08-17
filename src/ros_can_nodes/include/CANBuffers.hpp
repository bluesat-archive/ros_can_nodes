/*
 * Date Started: 20/10/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */

#ifndef CAN_BUFFERS_HPP
#define CAN_BUFFERS_HPP

#include <unordered_map>
#include <vector>
#include <cstdint>
#include <linux/can.h>

class CANBuffers {
    public:
        CANBuffers() {}

        void reset(const uint32_t key, const uint8_t expected_frames);

        void append(const uint32_t key, const uint8_t data[CAN_MAX_DLEN], const uint8_t data_len);

        /**
         * Returns whether the buffer for the given key is ready to be collected
         * ie. frames received equals expected frames
         */
        bool ready(const uint32_t key);

        const std::vector<uint8_t>& get(const uint32_t key);

    private:
        struct CANBuffer {
            std::vector<uint8_t> buf;
            uint8_t expected_frames;
            uint8_t received_frames;
        };
        
        std::unordered_map<uint32_t, class CANBuffer> buffers;

        CANBuffers(const CANBuffers&) = delete;
        void operator=(const CANBuffers&) = delete;
};
#endif // CAN_BUFFERS_HPP
