#ifndef ROSCANNODE_H
#define ROSCANNODE_H

#include "common.h"
#include "poll_manager.h"
#include "connection_manager.h"
#include "topic_manager.h"
#include "xmlrpc_manager.h"
#include "callback_queue_interface.h"
#include "rosout_appender.h"
#include <ros/callback_queue.h>
#include <std_msgs/String.h>

namespace roscan {

class NodeBackingCollection {
    public:
        //typedef std::vector<Publisher::ImplWPtr> V_PubImpl;
        //typedef std::vector<ServiceServer::ImplWPtr> V_SrvImpl;
        //typedef std::vector<Subscriber> V_Subscriber;
        //typedef std::vector<ServiceClient::ImplWPtr> V_SrvCImpl;
        //V_PubImpl pubs_;
        //V_SrvImpl srvs_;
        //V_Subscriber subs_;
        //V_SrvCImpl srv_cs_;

        boost::mutex mutex_;
};

class RosCanNode : public boost::enable_shared_from_this<RosCanNode> {
    public:
        RosCanNode(std::string name);
        ~RosCanNode();

        inline const std::string getName() const { return name_; }

        //Subscriber subscribe(ros::SubscribeOptions& ops);

        void spinOnce();

        //void subChatterCallback(const boost::shared_ptr<std_msgs::String const>&);

        ros::CallbackQueuePtr getInternalCallbackQueue();
        ros::CallbackQueue* getGlobalCallbackQueue() { return g_global_queue.get(); }

        void getAdvertisedTopics(V_string& topics);
        void getSubscribedTopics(V_string& topics);

        void start();

        const TopicManagerPtr& topic_manager();
        const ConnectionManagerPtr& connection_manager();
        const PollManagerPtr& poll_manager();
        const XMLRPCManagerPtr& xmlrpc_manager();

    private:
        std::string name_;

        TopicManagerPtr topicManager;
        ConnectionManagerPtr connectionManager;
        PollManagerPtr pollManager;
        XMLRPCManagerPtr xmlrpcManager;

        ros::CallbackQueueInterface* callback_queue_;
        NodeBackingCollection* collection_;

        ros::CallbackQueuePtr g_global_queue;
        ROSOutAppender* g_rosout_appender;
        ros::CallbackQueuePtr g_internal_callback_queue;
};

} // namespace roscan

#endif // ROSCANNODE_H
