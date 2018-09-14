/*
 * Date Started:
 * Original Author: Nuno Das Neves
 * Editors: Yiwei Han
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2018
 */

#ifndef ROSCANNODE_H
#define ROSCANNODE_H

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
#include <bitset>
#include <unordered_map>
#include <mutex>
#include <utility>
#include <ros_type_introspection/ros_introspection.hpp>
#include <topic_tools/shape_shifter.h>


#define MAX_NODES 16
#define MAX_TOPICS 128

namespace roscan {

    class NodeBackingCollection;

    class RosCanNode : public boost::enable_shared_from_this<RosCanNode> {
        public:
            RosCanNode(const std::string& name, const uint8_t id = 0);
            ~RosCanNode();

            uint8_t getID() const { return id_; }
            const std::string& getName() const { return name_; }

            void startSpinThread();
            void spin();

            // ==================================================
            //           CAN-facing methods and attributes
            // ==================================================

            void deregisterNode();
            void heartbeat();
            int registerSubscriber(const std::string& topic, const std::string& topic_type, const int request_tid = -1);
            int unregisterSubscriber(const uint8_t topic);
            PublisherPtr make_publisher(const std::string& topic, const std::string& topic_type);
            int advertiseTopic(const std::string& topic, const std::string& topic_type, const int request_tid = -1);
            int unregisterPublisher(const uint8_t topic);
            void publish(const uint8_t topicID, const std::vector<uint8_t>& value);
            int setParam(std::string key);
            int deleteParam(std::string key);
            int advertiseService(std::string service);
            int unregisterService(std::string service);
            int searchParam(std::string key);
            int subscribeParam(std::string key);
            int unsubscribeParam(std::string key);
            int hasParam(std::string key);
            int getParamNames();
            int getParam(std::string key);

            template<typename M>
            M convert_buf(const std::vector<uint8_t>& buf) {
                auto value = std::vector<uint8_t>(buf.cbegin(), buf.cend());
                std::cout << "message type size " << sizeof(M) << "\n";
                std::cout << "buffer size " << value.size() << "\n";
                if (sizeof(M) > value.size()) {
                    auto sz = sizeof(M) - value.size();
                    std::cout << "converting buffer: appending " << sz << " 0s to buffer\n";
                    value.insert(value.end(), sz, 0);
                }
                return *(M *)value.data();
            }

        private:
            const std::string name_;
            const uint8_t id_;

            bool g_started;
            volatile bool g_shutting_down;
            std::thread spinThread;

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

            // ==================================================
            //           ROS node methods and attributes
            // ==================================================
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

        private:
            void rosCanCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const uint8_t topic_num, const std::string& topic_name);

            std::mutex topicLock;
            std::bitset<MAX_TOPICS> topicIds;

            bool isZombie;
            int getFirstFreeTopic();
            std::unordered_map<uint8_t, std::pair<PublisherPtr, std::string>> publishers;
    };

} // namespace roscan

#endif // ROSCANNODE_H
