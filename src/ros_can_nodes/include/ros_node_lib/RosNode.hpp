/*
 * Date Started:
 * Original Author: Nuno Das Neves
 * Editors: Yiwei Han
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2018
 */

#ifndef ROSNODE_H
#define ROSNODE_H

#include "ros_node_lib/common.h"
#include "ros_node_lib/poll_manager.h"
#include "ros_node_lib/connection_manager.h"
#include "ros_node_lib/topic_manager.h"
#include "ros_node_lib/xmlrpc_manager.h"
#include "ros_node_lib/callback_queue_interface.h"
#include "ros_node_lib/rosout_appender.h"
#include "ros_node_lib/publisher.h"
#include "ros_node_lib/advertise_options.h"
#include "ros_node_lib/callback_queue.h"
#include <thread>
#include <boost/enable_shared_from_this.hpp>

namespace roscan {

    class NodeBackingCollection;

    class RosNode : public boost::enable_shared_from_this<RosNode> {
        public:
            RosNode(const std::string& name);
            ~RosNode();

            const std::string& getName() const { return name_; }

        private:
            const std::string name_;

            bool g_started;
            volatile bool g_shutting_down;

            TopicManagerPtr topicManager;
            ConnectionManagerPtr connectionManager;
            PollManagerPtr pollManager;
            XMLRPCManagerPtr xmlrpcManager;

            CallbackQueuePtr g_global_queue;
            ROSOutAppender *g_rosout_appender;
            CallbackQueuePtr g_internal_callback_queue;
            std::thread g_internal_queue_thread;
            void check_ipv6_environment();

            CallbackQueueInterface *callback_queue_;
            NodeBackingCollection *collection_;
            void internalCallbackQueueThreadFunc();

        public:
            void spinOnce();

            const CallbackQueuePtr& getInternalCallbackQueue();
            CallbackQueue *getGlobalCallbackQueue() const { return g_global_queue.get(); }

            void getAdvertisedTopics(std::vector<std::string>& topics);
            void getSubscribedTopics(std::vector<std::string>& topics);

            void start();
            void shutdown();

            const TopicManagerPtr& topic_manager();
            const ConnectionManagerPtr& connection_manager();
            const PollManagerPtr& poll_manager();
            const XMLRPCManagerPtr& xmlrpc_manager();

            // ==================================================
            //           Publisher and Subscriber templates
            // ==================================================

            PublisherPtr advertise(AdvertiseOptions& ops);
            SubscriberPtr subscribe(SubscribeOptions& ops);

            template <class M>
            PublisherPtr advertise(const std::string& topic, uint32_t queue_size, bool latch = false) {
                AdvertiseOptions ops;
                ops.template init<M>(topic, queue_size);
                ops.latch = latch;
                return advertise(ops);
            }

            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M), T* obj,
                                 const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M) const, T* obj,
                                 const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size,
                                 void (T::*fp)(const boost::shared_ptr<M const>&), T* obj,
                                 const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size,
                                 void (T::*fp)(const boost::shared_ptr<M const>&) const, T* obj,
                                 const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M),
                                 const boost::shared_ptr<T>& obj, const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
                ops.tracked_object = obj;
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M) const,
                                 const boost::shared_ptr<T>& obj, const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
                ops.tracked_object = obj;
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size,
                                 void (T::*fp)(const boost::shared_ptr<M const>&),
                                 const boost::shared_ptr<T>& obj, const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template init<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
                ops.tracked_object = obj;
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size,
                                 void (T::*fp)(const boost::shared_ptr<M const>&) const,
                                 const boost::shared_ptr<T>& obj, const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template init<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
                ops.tracked_object = obj;
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            template <class M>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size, void (*fp)(const boost::shared_ptr<M const>&), const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template init<M>(topic, queue_size, fp);
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            template <class M>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size, const boost::function<void(const boost::shared_ptr<M const>&)>& callback,
                                 const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(), const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template init<M>(topic, queue_size, callback);
                ops.tracked_object = tracked_object;
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            template <class M, class C>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size, const boost::function<void(C)>& callback,
                                 const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(), const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template initByFullCallbackType<C>(topic, queue_size, callback);
                ops.tracked_object = tracked_object;
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }
    };

} // namespace roscan

#endif // ROSNODE_H
