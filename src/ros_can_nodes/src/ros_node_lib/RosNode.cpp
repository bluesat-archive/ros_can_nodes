/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros_node_lib/common.h"
#include "ros_node_lib/RosNode.hpp"
#include "ros_node_lib/rosout_appender.h"
#include "ros_node_lib/advertise_options.h"
#include "ros_node_lib/callback_queue.h"
#include "ros_node_lib/publisher.h"
#include "ros_node_lib/subscriber.h"
#include "ros_node_lib/internal_timer_manager.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/transport/transport_tcp.h>
#include <xmlrpcpp/XmlRpcSocket.h>

namespace roscan {

    class NodeBackingCollection {
        public:
            // TODO stuff for ROS services
            typedef std::vector<PublisherPtr> V_Pubs;
            typedef std::vector<SubscriberPtr> V_Subs;
            V_Pubs pubs_;
            V_Subs subs_;

            std::mutex mutex_;
    };

    RosNode::RosNode(const std::string& name) : name_{name}, g_started{false}, g_shutting_down{false}, callback_queue_{0}, collection_{0}, is_zombie{false} {
        ROS_INFO_STREAM("Creating node " << name_);
        g_global_queue.reset(new CallbackQueue{});
        ROSCONSOLE_AUTOINIT;
        collection_ = new NodeBackingCollection{};
        ROS_INFO_STREAM("Created node " << name_);
    }

    RosNode::~RosNode() {
        shutdown();
        ROS_INFO_STREAM("Deleted node " << name_);
    }

    const CallbackQueuePtr& RosNode::getInternalCallbackQueue() {
        if (!g_internal_callback_queue) {
            g_internal_callback_queue.reset(new CallbackQueue{});
        }
        return g_internal_callback_queue;
    }

    void RosNode::getAdvertisedTopics(std::vector<std::string>& topics) {
        topic_manager()->getAdvertisedTopics(topics);
    }

    void RosNode::getSubscribedTopics(std::vector<std::string>& topics) {
        topic_manager()->getSubscribedTopics(topics);
    }

    const TopicManagerPtr& RosNode::topic_manager() {
        if (!topicManager) {
            topicManager.reset(new TopicManager{shared_from_this()});
        }
        return topicManager;
    }

    const ConnectionManagerPtr& RosNode::connection_manager() {
        if (!connectionManager) {
            connectionManager.reset(new ConnectionManager{shared_from_this()});
        }
        return connectionManager;
    }

    const PollManagerPtr& RosNode::poll_manager() {
        if (!pollManager) {
            pollManager.reset(new PollManager{});
        }
        return pollManager;
    }

    const XMLRPCManagerPtr& RosNode::xmlrpc_manager() {
        if (!xmlrpcManager) {
            xmlrpcManager.reset(new XMLRPCManager{});
        }
        return xmlrpcManager;
    }

    void RosNode::internalCallbackQueueThreadFunc() {
        ros::disableAllSignalsInThisThread();

        CallbackQueuePtr queue = getInternalCallbackQueue();

        while (!g_shutting_down) {
            queue->callAvailable(ros::WallDuration{0.1});
        }
    }

    void RosNode::start() {
        ROS_INFO_STREAM("Starting node " << name_);
        initInternalTimerManager();

        poll_manager();
        connection_manager()->start();
        topic_manager()->start();
        // xmlrpc manager must be started _after_ all functions are bound to it
        xmlrpc_manager()->start();

        ros::Time::init();

        g_rosout_appender = new ROSOutAppender{shared_from_this()};
        ros::console::register_appender(g_rosout_appender);

        g_internal_queue_thread = std::thread{&RosNode::internalCallbackQueueThreadFunc, this};
        getGlobalCallbackQueue()->enable();
        g_started = true;
        ROS_INFO_STREAM("Started node " << name_);
    }

    void RosNode::shutdown() {
        if (g_shutting_down) {
            return;
        }
        ROS_INFO_STREAM("Shutting down node " << name_);

        // Wait for spinning thread to end.
        is_zombie = true;
        if (spin_thread.joinable()) {
            spin_thread.join();
        }

        g_shutting_down = true;

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

        g_started = false;
        ROS_INFO_STREAM("Shut down node " << name_);
    }

    PublisherPtr RosNode::advertise(AdvertiseOptions& ops) {
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

    SubscriberPtr RosNode::subscribe(SubscribeOptions& ops) {
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

    void RosNode::spinOnce() {
        g_global_queue->callAvailable(ros::WallDuration{});
    }

    void RosNode::startSpinThread() {
        spin_thread = std::thread{[this](){
            ros::Rate r{100}; //100Hz
            while (!is_zombie) {
                spinOnce();
                r.sleep();
            }
        }};
    }

} // namespace roscan
