#include "RosCanNode.hpp"
#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <utility>
#include <ros_type_introspection/ros_introspection.hpp>
#include "shape_shifter.hpp"
#include "ROSCANConstants.hpp"
#include <linux/can.h>
#include "MessageBuffer.hpp"
#include "IntrospectionHelpers.hpp"
#include <std_msgs/Float64.h>
#include "owr_messages/pwm.h"
#include "owr_messages/motor.h"
#include "owr_messages/science.h"

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
        std::cout  << now->tm_hour << ":" << now->tm_min << std::endl;
    }

    int RosCanNode::registerSubscriber(const std::string& topic, const std::string& topic_type, const int request_tid) {
        std::cout << "node id " << (int)id_ << " subscribing to topic \"" << topic << "\" of type \"" << topic_type << "\"\n";

        int topic_num;
        if (request_tid >= 0 && request_tid < topicIds.size() && !topicIds[request_tid]) {
            topicIds[request_tid] = 1;
            topic_num = request_tid;
        } else {
            topic_num = getFirstFreeTopic();
        }
        std::cout << "topic_id" << (int)topic_num << "\n";

        if (topic_num >= 0) {
            boost::function<void(const ShapeShifter::ConstPtr&)> callback;
            callback = [this, topic_num, topic](const ShapeShifter::ConstPtr& msg) {
                rosCanCallback(msg, topic_num, topic);
            };
            subscribe(topic, 100, callback);

            //TODO: return the CODE to see if success or fail from the ROS master registerSubscriber
                //-2: ERROR: Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.
                //-1: FAILURE: Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.
                //0: SUCCESS: Method completed successfully
            return topic_num;
        } else {
            // TODO handle error, no free topic ids
            return -1;
        }
    }

    int RosCanNode::unregisterSubscriber(const uint8_t topic) {
        // TODO: at a later date, at this moment, just call unregister node
        std::cout << "node id " << (int)id_ << " unsubscribing from topic id " << (int)topic << "\n";
        return 0;
    }

    PublisherPtr RosCanNode::make_publisher(const std::string& topic, const std::string& topic_type) {
        constexpr uint32_t queue_size = 10;
        if (topic_type == "std_msgs/Float64") {
            return advertise<std_msgs::Float64>(topic, queue_size);
        } else if (topic_type == "owr_messages/pwm") {
            return advertise<owr_messages::pwm>(topic, queue_size);
        } else if (topic_type == "owr_messages/motor") {
            return advertise<owr_messages::motor>(topic, queue_size);
        } else if (topic_type == "owr_messages/science") {
            return advertise<owr_messages::science>(topic, queue_size);
        }
        // ...other message types if needed
        std::cout << "unimplemented advertise topic: " << topic_type << "\n";
        return PublisherPtr{};
    }

    int RosCanNode::advertiseTopic(const std::string& topic, const std::string& topic_type, const int request_tid) {
        std::cout << "node id " << (int)id_ << " advertising topic \"" << topic << "\" of type \"" << topic_type << "\"\n";

        int topic_num;
        if (request_tid >= 0 && request_tid < topicIds.size() && !topicIds[request_tid]) {
            topicIds[request_tid] = 1;
            topic_num = request_tid;
        } else {
            topic_num = getFirstFreeTopic();
        }

        if (topic_num >= 0) {
            PublisherPtr pub = make_publisher(topic, topic_type);
            if (!pub) {
                return -1;
            }
            publishers[(uint8_t)topic_num] = std::make_pair(pub, topic_type);

            //TODO: return the CODE to see if success or fail from the ROS master registerSubscriber
                //-2: ERROR: Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.
                //-1: FAILURE: Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.
                //0: SUCCESS: Method completed successfully
            return topic_num;
        } else {
            // TODO handle error, no free topic ids
            return -1;
        }

        //TODO: return the CODE to see if success or fail from the ROS master unregisterSubscriber
        return 0;
    }

    int RosCanNode::unregisterPublisher(const uint8_t topic) {
        // TODO: at a later date, at this moment, just call unregister node
        std::cout << "node id " << (int)id_ << " unadvertising topic id " << (int)topic << "\n";
        return 0;
    }

    void RosCanNode::publish(const uint8_t topicID, const std::vector<uint8_t>& buf) {
        const auto& topic_type = publishers[topicID].second;
        if (topic_type == "std_msgs/Float64") {
            auto msg = convert_buf<std_msgs::Float64>(buf);
            publishers[topicID].first->publish(msg);
        } else if (topic_type == "owr_messages/pwm") {
            auto msg = convert_buf<owr_messages::pwm>(buf);
            publishers[topicID].first->publish(msg);
        } else if (topic_type == "owr_messages/motor") {
            auto msg = convert_buf<owr_messages::motor>(buf);
            publishers[topicID].first->publish(msg);
        } else if (topic_type == "owr_messages/science") {
            auto msg = convert_buf<owr_messages::science>(buf);
            publishers[topicID].first->publish(msg);
        } else {
            return;
        }
        // ...other message types if needed
        std::cout << "node id " << (int)id_ << " published message on topic id " << (int)topicID << "\n";
    }

    int RosCanNode::setParam(std::string key) {

        //TODO: call ROS master setParam(caller_id, key, value) with callerId as first parameter
        std::cout << "Setting parameter" << '\n';

        //TODO: return the CODE to see if success or fail from the ROS master unregisterPublisher
        return 0;

    }

    int RosCanNode::deleteParam(std::string key) {

        //TODO: call ROS master deleteParam(caller_id, key) with callerId as first parameter
        std::cout << "Deleting parameter" << '\n';

        //TODO: return the CODE to see if success or fail from the ROS master deleteParam
        return 0;

    }

    int RosCanNode::advertiseService(std::string service) {

        //TODO: call ROS master registerService(caller_id, service, service_api, caller_api) with callerId as first parameter
        std::cout << "Registering service" << '\n';

        //TODO: return the CODE to see if success or fail from the ROS master registerService
        return 0;
    }

    int RosCanNode::unregisterService(std::string service) {

        //TODO: call ROS master unregisterService(caller_id, service, service_api) with callerId as first parameter
        std::cout << "Unregistering service" << '\n';

        //TODO: return the CODE to see if success or fail from the ROS master unregisterService
        return 0;

    }

    int RosCanNode::searchParam(std::string key) {

        //TODO: call ROS master searchParam(caller_id, key) with callerId as first parameter
        std::cout << "Searching for parameter key" << '\n';

        //TODO: return the CODE to see if found or not found from the ROS master searchParam
        return 0;
    }

    int RosCanNode::subscribeParam(std::string key) {

        //TODO: call ROS master subscribeParam(caller_id, caller_api, key) with callerId as first parameter
        std::cout << "Subscribing to parameter key" << '\n';

        //TODO: return the CODE to see if success or fail from the ROS master subscribeParam
        return 0;
    }

    int RosCanNode::unsubscribeParam(std::string key) {

        //TODO: call ROS master unsubscribeParam(caller_id, caller_api, key) with callerId as first parameter
        std::cout << "Unsubscribing to parameter key" << '\n';

        //TODO: return the CODE to see if success or fail from the ROS master unsubscribeParam
        return 0;
    }

    int RosCanNode::hasParam(std::string key) {

        //TODO: call ROS master unsubscribeParam(caller_id, caller_api, key) with callerId as first parameter
        std::cout << "Checking if parameter stored on server" << '\n';

        //TODO: return the CODE to see if success or fail from the ROS master hasParam
        return 0;
    }

    int RosCanNode::getParamNames() {

        //TODO: call ROS master getParamNames(caller_id) with callerId as parameter
        std::cout << "Getting list of all parameter names" << '\n';

        //TODO: return the CODE to see if success or fail from the ROS master getParamNames
        return 0;
    }

    int RosCanNode::getParam(std::string key) {

        //TODO: call ROS master getParam(caller_id, key) with callerId as parameter
        std::cout << "Getting a parameter" << '\n';

        //TODO: return the CODE to see if success or fail from the ROS master getParam
        return 0;
    }

    // ==================================================
    //               ROS Facing Methods
    // ==================================================

    void RosCanNode::rosCanCallback(const ShapeShifter::ConstPtr& msg, const uint8_t topicID, const std::string& topic_name) {
        ROS_INFO_STREAM("callback: topic_id = " << (int)topicID << " topic_name = " << topic_name);

        IntrospectionHelpers::registerMessage(msg, topic_name);
        const auto buf = IntrospectionHelpers::modify_buffer(msg->getDataType(), msg->raw_data(), msg->size());
        const auto msg_count = buf.size() / 8u + (buf.size() % 8u != 0u);
        ROS_INFO_STREAM("buf size " << buf.size() << " msg_count " << msg_count);

        canid_t header = 0;
        header |= CAN_EFF_FLAG;

        header |= (1 << ROSCANConstants::Common::bitshift_mode);
        header |= (0 << ROSCANConstants::Common::bitshift_priority);
        header |= (0 << ROSCANConstants::Common::bitshift_func);
        if (id_ == 1) {
	        header |= ((topicID * 2) << ROSCANConstants::ROSTopic::bitshift_topic_id);
        } else if (id_ == 0 ) {
            header |= ((topicID * 2 + 1) << ROSCANConstants::ROSTopic::bitshift_topic_id);
	    } else if (id_ == 4) {
            header |= ((topicID + 8) << ROSCANConstants::ROSTopic::bitshift_topic_id);
        } else {
            header |= (topicID << ROSCANConstants::ROSTopic::bitshift_topic_id);
        }
	    header |= (id_ << ROSCANConstants::ROSTopic::bitshift_nid);

        for (auto i = 0u;i < msg_count;++i) {
            can_frame frame;
            frame.can_id = header;

            frame.can_id |= (i & 0b11) << ROSCANConstants::ROSTopic::bitshift_msg_num;
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

        // // Second msg is empty data to indicate EOM
        // header |= (1 << ROSCANConstants::ROSTopic::bitshift_msg_num);
        // frame.can_id = header;
        // frame.can_dlc = 8;
        // *(uint64_t *)frame.data = 0;
        // MessageBuffer::instance().push(frame);
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
