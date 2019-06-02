/*
 * Date Started: 20/10/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */

#ifndef TOPICBUFFERS_H
#define TOPICBUFFERS_H

#include <unordered_map>
#include <vector>
#include <utility>
#include <cstdint>
#include <linux/can.h>


#define MAX_CAN_MSGS 32
#define TOPIC_BUFFER_SIZE (MAX_CAN_MSGS * CAN_MAX_DLEN)

class TopicBuffers {
    public:
        static TopicBuffers& instance();

        void reset(const short key, uint8_t can_frames);

        // returns whether the buffer for the topic is ready to be collected
        bool append(const short key, const uint8_t data[CAN_MAX_DLEN], const int dlc);

        std::vector<uint8_t>& get(const short key);

        // TODO: Possible Custom Hash function for unordered_map
        // TODO: Possible Custom Equality function for unordered_map
    private:
        TopicBuffers() {}

        // map key to pair containing the buffer array and
        // pair of expected can frames count and actual frames received
        std::unordered_map<short, std::pair<std::vector<uint8_t>, std::pair<uint8_t, uint8_t>>> topic_buffers;

        TopicBuffers(const TopicBuffers&) = delete;
        void operator=(const TopicBuffers&) = delete;
};
#endif // TOPICBUFFERS_H
