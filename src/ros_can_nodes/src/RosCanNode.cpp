#include "ros_node_lib/common.h"
#include "RosCanNode.hpp"
#include "ros_node_lib/rosout_appender.h"
#include "ros_node_lib/advertise_options.h"
#include "ros_node_lib/callback_queue.h"
#include "ros_node_lib/publisher.h"
#include "ros_node_lib/subscriber.h"
#include "ros_node_lib/internal_timer_manager.h"
#include <iostream>
#include <ros/console.h>
#include <ros/transport/transport_tcp.h>
#include <xmlrpcpp/XmlRpcSocket.h>
#include <unistd.h>
#include <cstring>
#include "ROSCANConstants.hpp"
#include <linux/can.h>
#include "MessageBuffer.hpp"

namespace roscan {

    class NodeBackingCollection {
        public:
            //typedef std::vector<Publisher::ImplWPtr> V_PubImpl;
            //typedef std::vector<ServiceServer::ImplWPtr> V_SrvImpl;
            //typedef std::vector<Subscriber::ImplWPtr> V_SubImpl;
            //typedef std::vector<ServiceClient::ImplWPtr> V_SrvCImpl;

            typedef std::vector<PublisherPtr> V_Pubs;
            //typedef std::vector<ServiceServer::ImplWPtr> V_SrvImpl;
            typedef std::vector<SubscriberPtr> V_Subs;
            //typedef std::vector<ServiceClient::ImplWPtr> V_SrvCImpl;
            V_Pubs pubs_;
            //V_SrvImpl srvs_;
            V_Subs subs_;
            //V_SrvCImpl srv_cs_;

            std::mutex mutex_;
    };

    void RosCanNode::check_ipv6_environment() {
        const auto env_ipv6 = getenv("ROS_IPV6");
        const auto use_ipv6 = env_ipv6 && strcmp(env_ipv6, "on") == 0;
        ros::TransportTCP::s_use_ipv6_ = use_ipv6;
        XmlRpc::XmlRpcSocket::s_use_ipv6_ = use_ipv6;
    }

    RosCanNode::RosCanNode(const std::string& name, const uint8_t id) : name_{"can_node/" + name}, id_{id}, isZombie{false}, g_started{false}, g_shutting_down{false}, callback_queue_{0}, collection_{0} {
        std::cout << "Creating node " << name_ << "\n";
        g_global_queue.reset(new CallbackQueue{});
        ROSCONSOLE_AUTOINIT;
        check_ipv6_environment();
        collection_ = new NodeBackingCollection{};
        std::cout << "Created node " << name_ << "\n";
    }

    RosCanNode::~RosCanNode() {
        shutdown();
        std::cout << "Deleted node " << name_ << "\n";
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

    int RosCanNode::registerSubscriber(const std::string& topic, const std::string& topic_type) {
        std::cout << "node id " << (int)id_ << " subscribing to topic \"" << topic << "\" of type \"" << topic_type << "\"\n";

        int topic_num = getFirstFreeTopic();

        if (topic_num >= 0) {
            boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
            callback = [this, topic_num, topic](const topic_tools::ShapeShifter::ConstPtr& msg) {
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
            return 0;
        }
    }

    int RosCanNode::unregisterSubscriber(const uint8_t topic) {
        // TODO: at a later date, at this moment, just call unregister node
        std::cout << "node id " << (int)id_ << " unsubscribing from topic id " << (int)topic << "\n";
        return 0;
    }

    int RosCanNode::advertiseTopic(const std::string& topic, const std::string& topic_type) {
        std::cout << "node id " << (int)id_ << " advertising topic \"" << topic << "\" of type \"" << topic_type << "\"\n";

        int topic_num = getFirstFreeTopic();
        if (topic_num >= 0) {

            //advertise<RosIntrospection::FlatMessage>(name, (uint32_t)10, false);

            //TODO: return the CODE to see if success or fail from the ROS master registerSubscriber
                //-2: ERROR: Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.
                //-1: FAILURE: Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.
                //0: SUCCESS: Method completed successfully
            return 0;
        } else {
            // TODO handle error, no free topic ids
            return 0;
        }

        //TODO: return the CODE to see if success or fail from the ROS master unregisterSubscriber
        return 0;
    }

    int RosCanNode::unregisterPublisher(const uint8_t topic) {
        // TODO: at a later date, at this moment, just call unregister node
        std::cout << "node id " << (int)id_ << " unadvertising topic id " << (int)topic << "\n";
        return 0;
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

    void RosCanNode::rosCanCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const uint8_t topicID, const std::string& topic_name) {
        ROS_INFO_STREAM("callback: topic_id = " << (int)topicID << " topic_name = " << topic_name);
        RosIntrospection::Parser parser;

        const std::string&  datatype   =  msg->getDataType();
        const std::string&  definition =  msg->getMessageDefinition();

        parser.registerMessageDefinition( topic_name,
                                          RosIntrospection::ROSType(datatype),
                                          definition);

        //reuse these objects to improve efficiency ("static" makes them persistent)
        static std::vector<uint8_t> buffer;
        static std::map<std::string,RosIntrospection::FlatMessage>   flat_containers;
        static std::map<std::string,RosIntrospection::RenamedValues> renamed_vectors;

        RosIntrospection::FlatMessage&   flat_container = flat_containers[topic_name];
        RosIntrospection::RenamedValues& renamed_values = renamed_vectors[topic_name];

        //copy raw memory into the buffer
        buffer.resize(msg->size());
        ros::serialization::OStream stream(buffer.data(), buffer.size());
        msg->write(stream);

        //deserialize and rename the vectors
        parser.deserializeIntoFlatContainer(topic_name, absl::Span<uint8_t>(buffer), &flat_container, 100);

        parser.applyNameTransform(topic_name, flat_container, &renamed_values);

        //print the content of the message
        printf("--------- %s ----------\n", topic_name.c_str());
        for (const auto& it: renamed_values) {
            const std::string& key = it.first;
            const RosIntrospection::Variant& value   = it.second;
            printf(" %s = %f\n", key.c_str(), value.convert<double>()); //convert into CAN message set
        }

        canid_t header = 0x0;

        header |= (1 << ROSCANConstants::Common::bitshift_mode);
        header |= (1 << ROSCANConstants::Common::bitshift_priority);
        header |= (0 << ROSCANConstants::Common::bitshift_func);
        header |= (0 << ROSCANConstants::Common::bitshift_seq);
        header |= ((topicID * 2) << ROSCANConstants::ROSTopic::bitshift_topic_id);
        header |= (id_ << ROSCANConstants::ROSTopic::bitshift_nid);
        header |= (0 << ROSCANConstants::ROSTopic::bitshift_msg_num);
        header |= (1 << ROSCANConstants::ROSTopic::bitshift_len);
        header |= CAN_EFF_FLAG;

        can_frame frame;

        frame.can_id = header;
        frame.can_dlc = 8;

        for (const auto& it: renamed_values) {
            *(double *)frame.data = it.second.convert<double>();
        }

        MessageBuffer::instance().push(frame);
    }

    const CallbackQueuePtr& RosCanNode::getInternalCallbackQueue() {
        if (!g_internal_callback_queue) {
            g_internal_callback_queue.reset(new CallbackQueue{});
        }
        return g_internal_callback_queue;
    }

    void RosCanNode::getAdvertisedTopics(std::vector<std::string>& topics) {
        topic_manager()->getAdvertisedTopics(topics);
    }

    void RosCanNode::getSubscribedTopics(std::vector<std::string>& topics) {
        topic_manager()->getSubscribedTopics(topics);
    }

    const TopicManagerPtr& RosCanNode::topic_manager() {
        if (!topicManager) {
            topicManager.reset(new TopicManager{shared_from_this()});
        }
        return topicManager;
    }

    const ConnectionManagerPtr& RosCanNode::connection_manager() {
        if (!connectionManager) {
            connectionManager.reset(new ConnectionManager{shared_from_this()});
        }
        return connectionManager;
    }

    const PollManagerPtr& RosCanNode::poll_manager() {
        if (!pollManager) {
            pollManager.reset(new PollManager{});
        }
        return pollManager;
    }

    const XMLRPCManagerPtr& RosCanNode::xmlrpc_manager() {
        if (!xmlrpcManager) {
            xmlrpcManager.reset(new XMLRPCManager{});
        }
        return xmlrpcManager;
    }

    void RosCanNode::internalCallbackQueueThreadFunc() {
        ros::disableAllSignalsInThisThread();

        CallbackQueuePtr queue = getInternalCallbackQueue();

        while (!g_shutting_down) {
            queue->callAvailable(ros::WallDuration{0.1});
        }
    }

    void RosCanNode::start() {
        std::cout << "Starting node " << name_ << "\n";
        initInternalTimerManager();

        poll_manager();
        connection_manager()->start();
        topic_manager()->start();
        // xmlrpc manager must be started _after_ all functions are bound to it
        xmlrpc_manager()->start();

        ros::Time::init();

        g_rosout_appender = new ROSOutAppender{shared_from_this()};
        ros::console::register_appender(g_rosout_appender);

        g_internal_queue_thread = std::thread{&RosCanNode::internalCallbackQueueThreadFunc, this};
        getGlobalCallbackQueue()->enable();
        g_started = true;
        std::cout << "Started node " << name_ << "\n";
    }

    void RosCanNode::shutdown() {
        if (g_shutting_down) {
            return;
        }
        std::cout << "Shutting down node " << name_ << "\n";

        // Wait for spinning thread to end.
        isZombie = true;
        if (spinThread.joinable()) {
            spinThread.join();
        }

        g_shutting_down = true;
        //ros::console::shutdown();

        g_global_queue->disable();
        g_global_queue->clear();

        if (g_internal_queue_thread.joinable()) {
            g_internal_queue_thread.join();
        }

        delete collection_;

        g_rosout_appender = 0;

        if (g_started) {
            topic_manager()->shutdown();
            poll_manager()->shutdown();
            connection_manager()->shutdown();
            xmlrpc_manager()->shutdown();
        }
        //ros::Time::shutdown();
        g_started = false;
        std::cout << "Shut down node " << name_ << "\n";
    }

    PublisherPtr RosCanNode::advertise(AdvertiseOptions& ops) {
        if (ops.callback_queue == 0) {
            if (callback_queue_) {
                ops.callback_queue = callback_queue_;
            } else {
                ops.callback_queue = getGlobalCallbackQueue();
            }
        }

        SubscriberCallbacksPtr callbacks{boost::make_shared<SubscriberCallbacks>(ops.connect_cb, ops.disconnect_cb, ops.tracked_object, ops.callback_queue)};

        if (topic_manager()->advertise(ops, callbacks)) {
            const auto pub = boost::make_shared<Publisher>(ops.topic, shared_from_this(), ops.md5sum, ops.datatype, callbacks);

            {
                std::lock_guard<std::mutex> lock{collection_->mutex_};
                collection_->pubs_.push_back(pub);
            }
            return pub;
        }
        return boost::make_shared<Publisher>();
    }

    SubscriberPtr RosCanNode::subscribe(SubscribeOptions& ops) {
        if (ops.callback_queue == 0) {
            if (callback_queue_) {
                ops.callback_queue = callback_queue_;
            } else {
                ops.callback_queue = getGlobalCallbackQueue();
            }
        }

        if (topicManager->subscribe(ops)) {
            const auto sub = boost::make_shared<Subscriber>(ops.topic, shared_from_this(), ops.helper);

            {
                std::lock_guard<std::mutex> lock{collection_->mutex_};
                collection_->subs_.push_back(sub);
            }
            return sub;
        }
        return boost::make_shared<Subscriber>();
    }

    void RosCanNode::spinOnce() {
        g_global_queue->callAvailable(ros::WallDuration{});
    }

    void RosCanNode::spin() {
        ros::Rate r{100}; //100Hz
        while (!isZombie) {
            spinOnce();
            r.sleep();
        }
    }

    void RosCanNode::startSpinThread() {
        spinThread = std::thread{&RosCanNode::spin, this};
    }

    int RosCanNode::getFirstFreeTopic() {
        // return the position of the first unset bit
        return ffs(~topicIds.to_ulong()) - 1;
    }


} // namespace roscan
