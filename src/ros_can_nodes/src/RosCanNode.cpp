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
#include <ros/file_log.h>
#include <ros/transport/transport_tcp.h>
#include <xmlrpcpp/XmlRpcSocket.h>
#include <unistd.h>

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

            boost::mutex mutex_;
    };

    void RosCanNode::check_ipv6_environment() {
        char* env_ipv6 = NULL;
        env_ipv6 = getenv("ROS_IPV6");

        bool use_ipv6 = (env_ipv6 && strcmp(env_ipv6, "on") == 0);
        ros::TransportTCP::s_use_ipv6_ = use_ipv6;
        XmlRpc::XmlRpcSocket::s_use_ipv6_ = use_ipv6;
    }

    RosCanNode::RosCanNode(std::string name, uint8_t id) : name_("can_node/" + name), id_(id), isZombie(false), g_started(false), g_shutting_down(false), callback_queue_(0), collection_(0) {

        std::cout << "Creating node " << name_ << "\n";
        g_global_queue.reset(new CallbackQueue);
        ROSCONSOLE_AUTOINIT;
        check_ipv6_environment();
        collection_ = new NodeBackingCollection;

        std::cout << "Created node " << name_ << "\n";
    }

    RosCanNode::~RosCanNode() {

        // Wait for spinning thread to end.
        this->isZombie = true;
        this->spinThread.join();

        {
            boost::mutex::scoped_lock nodeListLock(nodeListMutex);
            nodeList[this->getID()] = NULL;
        }

        shutdown();
        std::cout << "Deleting node " << name_ << "\n";
        delete collection_;
        std::cout << "Deleted node " << name_ << "\n";
    }

    // ==================================================
    //               CAN Facing Methods
    // ==================================================

    static RosCanNode * getNode(uint8_t id) {
        boost::mutex::scoped_lock nodeListLock(nodeListMutex);
        return nodeList[id];
    }

    static void registerNode(std::string name, uint8_t hashName) {
        uint8_t index = 0;
        //get the first id

        {
            boost::mutex::scoped_lock nodeListLock(nodeListMutex);

            while (index < MAX_NODES && nodeList[index] != NULL) {
                index++;
                if (index >= MAX_NODES) {
                    std::cout << "No available ids" << '\n';
                    break;
                }
            }

            if(index < MAX_NODES) {
                //TODO: take name from frame data
                RosCanNode *node = new RosCanNode(name, index);
                nodeList[index] = node;

                // if successful, create thread to loop spinOnce for any subscribers
                node->spinThread(&RosCanNode::spin, node);
            }
        }

        // TODO send response?
    }

    void RosCanNode::deregisterNode() {
        {
            boost::mutex::scoped_lock nodeListLock(nodeListMutex);
            nodeList[this->getID()] = NULL;
        }
        delete this;

        //TODO send response, will need to store nodeid if so.
    }

    //NOTE: do at a later date
    void RosCanNode::heartbeat() {
        time_t t = time(0);
        struct tm * now = localtime( & t );
        std::cout  << now->tm_hour << ":" << now->tm_min << std::endl;
    }

    int RosCanNode::registerSubscriber(std::string topic, std::string topic_type) {
        std::cout << "Calling subscriber" << '\n';

        std::string name = "/" + topic;
        int topic_num = getFirstFreeTopic();

        if(topic_num >= 0){

            this->subscribe(name, 1000, boost::bind(rosCanCallback, _1, topic_num);

            //TODO: return the CODE to see if success or fail from the ROS master registerSubscriber
                //-2: ERROR: Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.
                //-1: FAILURE: Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.
                //0: SUCCESS: Method completed successfully
            return 0;
        } else {
            // TODO handle error, no free topic ids
            return 0;
        }
    }

    int RosCanNode::unregisterSubscriber(uint8_t topic) {
        // TODO: at a later date, at this moment, just call unregister node
        std::cout << "Unregistering subscriber" << '\n';

        return 0;
    }

    int RosCanNode::advertiseTopic(std::string topic, std::string topic_type) {
        std::cout << "Advertising topic" << '\n';

        std::string name = "/" + topic;
        int topic_num = getFirstFreeTopic();

        if(topic_num >= 0){

            this->advertise<RosIntrospection::FlatMessage>(name, (uint32_t)10, false);

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

    int RosCanNode::unregisterPublisher(uint8_t topic) {
        // TODO: at a later date, at this moment, just call unregister node

        std::cout << "Unregistering publisher" << '\n';

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

    void rosCanCallback(const RosIntrospection::FlatMessage& msg, uint8_t topicID){
        //TODO: call Syam's FlatMessage conversion, then put on can bus
    }

    CallbackQueuePtr RosCanNode::getInternalCallbackQueue() {
        if (!g_internal_callback_queue) {
            g_internal_callback_queue.reset(new CallbackQueue);
        }
        return g_internal_callback_queue;
    }

    void RosCanNode::getAdvertisedTopics(V_string& topics) {
        topic_manager()->getAdvertisedTopics(topics);
    }

    void RosCanNode::getSubscribedTopics(V_string& topics) {
        topic_manager()->getSubscribedTopics(topics);
    }

    const TopicManagerPtr& RosCanNode::topic_manager() {
        if (!topicManager) {
            topicManager.reset(new TopicManager(shared_from_this()));
        }
        return topicManager;
    }

    const ConnectionManagerPtr& RosCanNode::connection_manager() {
        if (!connectionManager) {
            connectionManager.reset(new ConnectionManager(shared_from_this()));
        }
        return connectionManager;
    }

    const PollManagerPtr& RosCanNode::poll_manager() {
        if (!pollManager) {
            pollManager.reset(new PollManager());
        }
        return pollManager;
    }

    const XMLRPCManagerPtr& RosCanNode::xmlrpc_manager() {
        if (!xmlrpcManager) {
            xmlrpcManager.reset(new XMLRPCManager());
        }
        return xmlrpcManager;
    }

    void RosCanNode::internalCallbackQueueThreadFunc() {
        ros::disableAllSignalsInThisThread();

        CallbackQueuePtr queue = getInternalCallbackQueue();

        while (!g_shutting_down) {
            queue->callAvailable(ros::WallDuration(0.1));
        }
    }

    void RosCanNode::start() {
        std::cout << "Starting node " << name_ << "\n";
        initInternalTimerManager();

        poll_manager()->start();
        connection_manager()->start();
        topic_manager()->start();
        // xmlrpc manager must be started _after_ all functions are bound to it
        xmlrpc_manager()->start();

        ros::Time::init();

        g_rosout_appender = new ROSOutAppender(shared_from_this());
        ros::console::register_appender(g_rosout_appender);

        g_internal_queue_thread = boost::thread(&RosCanNode::internalCallbackQueueThreadFunc, this);
        getGlobalCallbackQueue()->enable();
        g_started = true;
        std::cout << "Started node " << name_ << "\n";
    }

    void RosCanNode::shutdown() {
        if (g_shutting_down) {
            return;
        }
        std::cout << "Shutting down node " << name_ << "\n";
        g_shutting_down = true;
        //ros::console::shutdown();

        g_global_queue->disable();
        g_global_queue->clear();

        if (g_internal_queue_thread.get_id() != boost::this_thread::get_id()) {
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

        SubscriberCallbacksPtr callbacks(boost::make_shared<SubscriberCallbacks>(ops.connect_cb, ops.disconnect_cb, ops.tracked_object, ops.callback_queue));

        if (topic_manager()->advertise(ops, callbacks)) {
            PublisherPtr pub = boost::make_shared<Publisher>(ops.topic, shared_from_this(), ops.md5sum, ops.datatype, callbacks);

            {
                boost::mutex::scoped_lock lock(collection_->mutex_);
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
            SubscriberPtr sub = boost::make_shared<Subscriber>(ops.topic, shared_from_this(), ops.helper);

            {
                boost::mutex::scoped_lock lock(collection_->mutex_);
                collection_->subs_.push_back(sub);
            }
            return sub;
        }
        return boost::make_shared<Subscriber>();
    }

    void RosCanNode::spinOnce() {
        g_global_queue->callAvailable(ros::WallDuration());
    }

    void RosCanNode::spin(){
        ros::Rate r(100); //100Hz
        while (!isZombie) {
            this->spinOnce();
            r.sleep();
        }
    }

    int RosCanNode::getFirstFreeTopic(){
        // TODO: make this more efficient
        int i;
        for(i = 0; i < topicIds.size(); i++){
            if(!topicIds[i]){
                return i;
            }
        }

        return -1;
    }


} // namespace roscan
