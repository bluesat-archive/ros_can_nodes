#include "RosCanNode.hpp"
#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <utility>
#include <ros_type_introspection/ros_introspection.hpp>
#include <ros_type_introspection/utils/shape_shifter.hpp>
#include "ROSCANConstants.hpp"
#include <linux/can.h>
#include "MessageBuffer.hpp"
#include "IntrospectionHelpers.hpp"
#include "message_properties_map.hpp"

namespace roscan {
    using RosIntrospection::ShapeShifter;

    RosCanNode::RosCanNode(const std::string& name, const uint8_t id) : RosNode{"can_node/" + name}, id_{id} {
        ROS_INFO("CAN node name: %s", name_.c_str());
        ROS_INFO("Node id: %d", id_);
    }

    // ==================================================
    //               CAN Facing Methods
    // ==================================================

    //NOTE: do at a later date
    void RosCanNode::heartbeat() {
        time_t t = time(0);
        tm *now = localtime(&t);
        ROS_INFO("%02d:%02d", now->tm_hour, now->tm_min);
    }

    int RosCanNode::registerSubscriber(const std::string& topic, const std::string& topic_type, const int request_tid) {
        ROS_INFO("node id %d subscribing to topic \"%s\" of type \"%s\"", id_, topic.c_str(), topic_type.c_str());

        int topicID;
        if (request_tid >= 0 && request_tid < topicIds.size() && !topicIds[request_tid]) {
            topicIds[request_tid] = 1;
            topicID = request_tid;
        } else {
            topicID = getFirstFreeTopic();
        }
        ROS_INFO("got topic_id %d", topicID);

        if (topicID >= 0) {
            boost::function<void(const ShapeShifter::ConstPtr&)> callback;
            callback = [this, topicID, topic](const ShapeShifter::ConstPtr& msg) {
                rosCanCallback(msg, topicID, topic);
            };
            subscribe(topic, 100, callback);

            //TODO: return the CODE to see if success or fail from the ROS master registerSubscriber
                //-2: ERROR: Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.
                //-1: FAILURE: Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.
                //0: SUCCESS: Method completed successfully
            return topicID;
        } else {
            // TODO handle error, no free topic ids
            return -1;
        }
    }

    int RosCanNode::unregisterSubscriber(const uint8_t topicID) {
        // TODO: at a later date, at this moment, just call unregister node
        ROS_INFO("node id %d unsubscribing from topic id %d", id_, topicID);
        return 0;
    }

    PublisherPtr RosCanNode::make_publisher(const std::string& topic, const std::string& topic_type) {
        constexpr uint32_t queue_size = 10;
        AdvertiseOptions opts{
            topic,
            queue_size,
            message_properties_map.at(topic_type).md5sum,
            topic_type,
            message_properties_map.at(topic_type).definition
        };
        return advertise(opts);
    }

    int RosCanNode::advertiseTopic(const std::string& topic, const std::string& topic_type, const int request_tid) {
        ROS_INFO("node id %d advertising topic \"%s\" of type \"%s\"", id_, topic.c_str(), topic_type.c_str());

        int topicID;
        if (request_tid >= 0 && request_tid < topicIds.size() && !topicIds[request_tid]) {
            topicIds[request_tid] = 1;
            topicID = request_tid;
        } else {
            topicID = getFirstFreeTopic();
        }
        ROS_INFO("got topic_id %d", topicID);

        if (topicID >= 0) {
            PublisherPtr pub = make_publisher(topic, topic_type);
            if (!pub) {
                return -1;
            }
            publishers[static_cast<uint8_t>(topicID)] = pub;

            //TODO: return the CODE to see if success or fail from the ROS master registerSubscriber
                //-2: ERROR: Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.
                //-1: FAILURE: Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.
                //0: SUCCESS: Method completed successfully
            return topicID;
        } else {
            // TODO handle error, no free topic ids
            return -1;
        }
    }

    int RosCanNode::unregisterPublisher(const uint8_t topicID) {
        // TODO: at a later date, at this moment, just call unregister node
        ROS_INFO("node id %d unadvertising topic id %d", id_, topicID);
        return 0;
    }

    void RosCanNode::publish(const uint8_t topicID, std::vector<uint8_t>& data) {
        auto& pub = publishers[topicID];
        const auto topic_type = pub->getDatatype();
        ros::serialization::OStream stream{data.data(), static_cast<uint32_t>(data.size())};
        ShapeShifter shape_shifter;
        shape_shifter.morph(
            message_properties_map.at(topic_type).md5sum,
            topic_type,
            message_properties_map.at(topic_type).definition
        );
        shape_shifter.read(stream);
        pub->publish(shape_shifter);
        ROS_INFO("node id %d published message on topic id %d", id_, topicID);
    }

    int RosCanNode::setParam(std::string key) {

        //TODO: call ROS master setParam(caller_id, key, value) with callerId as first parameter
        ROS_INFO("Setting parameter");

        //TODO: return the CODE to see if success or fail from the ROS master unregisterPublisher
        return 0;

    }

    int RosCanNode::deleteParam(std::string key) {

        //TODO: call ROS master deleteParam(caller_id, key) with callerId as first parameter
        ROS_INFO("Deleting parameter");

        //TODO: return the CODE to see if success or fail from the ROS master deleteParam
        return 0;

    }

    int RosCanNode::advertiseService(std::string service) {

        //TODO: call ROS master registerService(caller_id, service, service_api, caller_api) with callerId as first parameter
        ROS_INFO("Registering service");

        //TODO: return the CODE to see if success or fail from the ROS master registerService
        return 0;
    }

    int RosCanNode::unregisterService(std::string service) {

        //TODO: call ROS master unregisterService(caller_id, service, service_api) with callerId as first parameter
        ROS_INFO("Unregistering service");

        //TODO: return the CODE to see if success or fail from the ROS master unregisterService
        return 0;

    }

    int RosCanNode::searchParam(std::string key) {

        //TODO: call ROS master searchParam(caller_id, key) with callerId as first parameter
        ROS_INFO("Searching for parameter key");

        //TODO: return the CODE to see if found or not found from the ROS master searchParam
        return 0;
    }

    int RosCanNode::subscribeParam(std::string key) {

        //TODO: call ROS master subscribeParam(caller_id, caller_api, key) with callerId as first parameter
        ROS_INFO("Subscribing to parameter key");

        //TODO: return the CODE to see if success or fail from the ROS master subscribeParam
        return 0;
    }

    int RosCanNode::unsubscribeParam(std::string key) {

        //TODO: call ROS master unsubscribeParam(caller_id, caller_api, key) with callerId as first parameter
        ROS_INFO("Unsubscribing to parameter key");

        //TODO: return the CODE to see if success or fail from the ROS master unsubscribeParam
        return 0;
    }

    int RosCanNode::hasParam(std::string key) {

        //TODO: call ROS master unsubscribeParam(caller_id, caller_api, key) with callerId as first parameter
        ROS_INFO("Checking if parameter stored on server");

        //TODO: return the CODE to see if success or fail from the ROS master hasParam
        return 0;
    }

    int RosCanNode::getParamNames() {

        //TODO: call ROS master getParamNames(caller_id) with callerId as parameter
        ROS_INFO("Getting list of all parameter names");

        //TODO: return the CODE to see if success or fail from the ROS master getParamNames
        return 0;
    }

    int RosCanNode::getParam(std::string key) {

        //TODO: call ROS master getParam(caller_id, key) with callerId as parameter
        ROS_INFO("Getting a parameter");

        //TODO: return the CODE to see if success or fail from the ROS master getParam
        return 0;
    }

    // ==================================================
    //               ROS Facing Methods
    // ==================================================

    void RosCanNode::rosCanCallback(const ShapeShifter::ConstPtr& msg, const uint8_t topicID, const std::string& topic_name) {
        ROS_INFO("callback: topic_id = %d topic_name = %s", topicID, topic_name.c_str());

        const auto buf = IntrospectionHelpers::to_can_buf(msg->getDataType(), msg->raw_data(), msg->size());
        const auto msg_count = buf.size() / 8u + (buf.size() % 8u != 0u);
        ROS_INFO("buf size %lu msg_count %lu", buf.size(), msg_count);

        canid_t header = 0;
        header |= CAN_EFF_FLAG;

        ROS_INFO("can topic id 0x%x", topicID);
        header |= (1 << ROSCANConstants::Common::bitshift_mode);
        header |= (0 << ROSCANConstants::Common::bitshift_priority);
        header |= (0 << ROSCANConstants::Common::bitshift_func);
        header |= (((uint32_t)topicID) << ROSCANConstants::ROSTopic::bitshift_topic_id);
        ROS_INFO("topic id at position 0x%x", (((uint32_t)topicID) << ROSCANConstants::ROSTopic::bitshift_topic_id));
	    header |= (id_ << ROSCANConstants::ROSTopic::bitshift_nid);
        header |= (((msg_num = ((msg_num + 1)  % 3)) & ROSCANConstants::ROSTopic::bitmask_msg_num) << ROSCANConstants::ROSTopic::bitshift_msg_num);
        header |= CAN_EFF_FLAG;

        for (auto i = 0u;i < msg_count;++i) {
            can_frame frame;
            frame.can_id = header;

            if (i >= 0b111) {
                frame.can_id |= (0b111 << ROSCANConstants::Common::bitshift_seq);
                frame.can_id |= i << ROSCANConstants::ROSTopic::bitshift_len;
            } else {
                frame.can_id |= (i << ROSCANConstants::Common::bitshift_seq);
                frame.can_id |= msg_count << ROSCANConstants::ROSTopic::bitshift_len;
            }
            const auto start_offset = i * CAN_MAX_DLC;
            frame.can_dlc = std::min(buf.size() - start_offset, static_cast<uint64_t>(CAN_MAX_DLC));
            const auto begin = buf.cbegin() + start_offset;
            const auto end = begin + frame.can_dlc;
            std::copy(begin, end, frame.data);
            MessageBuffer::instance().push(frame);
        }
    }

    int RosCanNode::getFirstFreeTopic() {
        // return the position of the first unset bit
        int id = ffs(~topicIds.to_ulong()) - 1;

        if (id >= 0) {
            topicIds[id] = 1;
        }

        return id;
    }

} // namespace roscan
