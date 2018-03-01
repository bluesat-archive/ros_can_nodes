#include "common.h"
#include "RosCanNode.h"
#include "rosout_appender.h"
#include <iostream>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <unistd.h>

namespace roscan {

RosCanNode::RosCanNode(std::string name)
    : name_("/" + name), callback_queue_(0), collection_(new NodeBackingCollection) {
    g_global_queue.reset(new ros::CallbackQueue);
}

RosCanNode::~RosCanNode() {
    delete collection_;
}

ros::CallbackQueuePtr RosCanNode::getInternalCallbackQueue() {
    if (!g_internal_callback_queue) {
        g_internal_callback_queue.reset(new ros::CallbackQueue());
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

void RosCanNode::start() {
    poll_manager()->start();
    connection_manager()->start();
    topic_manager()->start();
    // xmlrpc manager must be started _after_ all functions are bound to it
    xmlrpc_manager()->start();

    g_rosout_appender = new ROSOutAppender(shared_from_this());
    ros::console::register_appender(g_rosout_appender);
}

/*
void RosCanNode::subChatterCallback(const boost::shared_ptr<std_msgs::String const>& msg) {
    std::cout << "received " << msg->data << std::endl;
}

Subscriber RosCanNode::subscribe(ros::SubscribeOptions& ops) {
    if (ops.callback_queue == 0) {
        if (callback_queue_) {
            ops.callback_queue = callback_queue_;
        } else {
            std::cout << "argh no callback queue\n";
            return Subscriber();
        }
    }

    if (topicManager->subscribe(ops)) {
        Subscriber sub(ops.topic, boost::make_shared<RosCanNode>(*this), ops.helper);

        {
            boost::mutex::scoped_lock lock(collection_->mutex_);
            collection_->subs_.push_back(sub);
        }

        return sub;
    }

    return Subscriber();
}

void RosCanNode::spinOnce() {
    ((ros::CallbackQueue*)callback_queue_)->callAvailable(ros::WallDuration());
}
*/

} // namespace roscan
