#include "common.h"
#include "RosCanNode.h"
#include <iostream>
//#include "subscriber.h"
//#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>
#include <unistd.h>

namespace roscan {

RosCanNode::RosCanNode(std::string name) : name_(name) {
    g_internal_callback_queue.reset(new ros::CallbackQueue);

    //collection_ = new NodeBackingCollection;
    //callback_queue_ = new ros::CallbackQueue;
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
    std::cout << "  Starting poll manager\n";
    poll_manager()->start();
    std::cout << "  Starting connection manager\n";
    connection_manager()->start();
    std::cout << "  Starting topic manager\n";
    topic_manager()->start();
    // xmlrpc manager must be started _after_ all functions are bound to it
    std::cout << "  Starting xmlrpc manager\n";
    xmlrpc_manager()->start();
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
