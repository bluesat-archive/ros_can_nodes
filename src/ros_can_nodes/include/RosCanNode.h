#ifndef ROSCANNODE_H
#define ROSCANNODE_H

#include "common.h"
#include "poll_manager.h"
#include "connection_manager.h"
#include "topic_manager.h"
#include "xmlrpc_manager.h"
#include "callback_queue_interface.h"
#include "rosout_appender.h"
#include "publisher.h"
#include "advertise_options.h"
#include "callback_queue.h"
#include <std_msgs/String.h>

namespace roscan {

class NodeBackingCollection;

class RosCanNode : public boost::enable_shared_from_this<RosCanNode> {
    public:
        RosCanNode(std::string name);
        ~RosCanNode();

        inline const std::string getName() const { return name_; }

        //Subscriber subscribe(ros::SubscribeOptions& ops);

        void spinOnce();

        //void subChatterCallback(const boost::shared_ptr<std_msgs::String const>&);

        CallbackQueuePtr getInternalCallbackQueue();
        CallbackQueue* getGlobalCallbackQueue() { return g_global_queue.get(); }

        void getAdvertisedTopics(V_string& topics);
        void getSubscribedTopics(V_string& topics);

        void start();
        void shutdown();

        const TopicManagerPtr& topic_manager();
        const ConnectionManagerPtr& connection_manager();
        const PollManagerPtr& poll_manager();
        const XMLRPCManagerPtr& xmlrpc_manager();

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
        std::string name_;

        bool g_started;
        volatile bool g_shutting_down;

        TopicManagerPtr topicManager;
        ConnectionManagerPtr connectionManager;
        PollManagerPtr pollManager;
        XMLRPCManagerPtr xmlrpcManager;

        CallbackQueuePtr g_global_queue;
        ROSOutAppender* g_rosout_appender;
        CallbackQueuePtr g_internal_callback_queue;
        boost::thread g_internal_queue_thread;
        void check_ipv6_environment();

        CallbackQueueInterface* callback_queue_;
        NodeBackingCollection* collection_;
        void internalCallbackQueueThreadFunc();
};

} // namespace roscan

#endif // ROSCANNODE_H
