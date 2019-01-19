/**
 * Date Started:
 * Original Author: Nuno Das Neves
 * Editors: Yiwei Han
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the BSB License. Copyright BLUEsat UNSW, 2018
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

            /**
             * Returns the name of this node.
             */
            const std::string& getName() const { return name_; }

        protected:
            std::string name_;

        private:
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

            CallbackQueueInterface *callback_queue_;
            NodeBackingCollection *collection_;
            void internalCallbackQueueThreadFunc();

            // spinner thread
            volatile bool is_zombie;
            std::thread spin_thread;

        public:
            void spinOnce();
            void startSpinThread();

            const CallbackQueuePtr& getInternalCallbackQueue();

            /**
             * \brief Returns a pointer to the global default callback queue.
             *
             * This is the queue that all callbacks get added to unless a different one is specified, either in the node
             * or in the individual subscribe()/advertise()/etc. functions.
             */
            CallbackQueue *getGlobalCallbackQueue() const { return g_global_queue.get(); }

            /**
             * \brief Get the list of topics advertised by this node
             *
             * @param[out] topics The advertised topics
             */
            void getAdvertisedTopics(std::vector<std::string>& topics);

            /**
             * \brief Get the list of topics subscribed to by this node
             *
             * @param[out] The subscribed topics
             */
            void getSubscribedTopics(std::vector<std::string>& topics);

            /**
             * Starts all the internal managers needed for this node to run.
             * Must be called before the node starts spinning.
             */
            void start();

            /**
             * Shuts down all internal managers and the node's spinning thread.
             * Must be called before the node fully destructs.
             */
            void shutdown();

            /**
             * Returns shared pointers to the internal managers.
             * TODO make manager classes friends of RosNode so that these can be private.
             */
            const TopicManagerPtr& topic_manager();
            const ConnectionManagerPtr& connection_manager();
            const PollManagerPtr& poll_manager();
            const XMLRPCManagerPtr& xmlrpc_manager();

            /** \brief Advertise a topic, with full range of AdvertiseOptions
             *
             * This call connects to the master to publicize that the node will be
             * publishing messages on the given topic.  This method returns a Publisher that allows you to
             * publish a message on this topic.
             *
             * This is an advanced version advertise() that exposes all options (through the AdvertiseOptions structure)
             *
             * \param ops Advertise options to use
             * \return On success, a PublisherPtr that, when it goes out of scope, will automatically release a reference
             * on this advertisement.  On failure, an empty PublisherPtr which can be checked with:
            \verbatim
            ros::AdvertiseOptions ops;
            PublisherPtr pub = node.advertise(ops);
            if (pub) {...}  // Enter if publisher is valid
            \endverbatim
             *
             * \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
             */
            PublisherPtr advertise(AdvertiseOptions& ops);

            /**
             * \brief Subscribe to a topic, version with full range of SubscribeOptions
             *
             * This method connects to the master to register interest in a given
             * topic.  The node will automatically be connected with publishers on
             * this topic.  On each message receipt, fp is invoked and passed a shared pointer
             * to the message received.  This message should \b not be changed in place, as it
             * is shared with any other subscriptions to this topic.
             *
             * This version of subscribe allows the full range of options, exposed through the SubscribeOptions class
             *
             * \param ops Subscribe options
             * \return On success, a SubscriberPtr that, when all copies of it go out of scope, will unsubscribe from this topic.
             * On failure, an empty SubscriberPtr which can be checked with:
            \verbatim
            SubscribeOptions ops;
            SubscriberPtr sub = node.subscribe(ops);
            if (sub) {} // Enter if subscriber is valid
            \endverbatim
             *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
             *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
             */
            SubscriberPtr subscribe(SubscribeOptions& ops);

            // ==================================================
            //           Publisher and Subscriber templates
            // ==================================================

            /**
             * \brief Advertise a topic, simple version
             *
            \verbatim
            PublisherPtr pub = node.advertise<std_msgs::Empty>("my_topic", 1);
            \endverbatim
             */
            template <class M>
            PublisherPtr advertise(const std::string& topic, uint32_t queue_size, bool latch = false) {
                AdvertiseOptions ops;
                ops.template init<M>(topic, queue_size);
                ops.latch = latch;
                return advertise(ops);
            }

            /**
             * \brief Subscribe to a topic, version for class member function with bare pointer
             *
            \verbatim
            void Foo::callback(const std_msgs::Empty::ConstPtr& message){...}
            Foo foo_object;
            SubscriberPtr sub = node.subscribe("my_topic", 1, &Foo::callback, &foo_object);
            \endverbatim
             */
            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M), T* obj,
                                 const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            /// and the const version
            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M) const, T* obj,
                                 const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            /**
             * \brief Subscribe to a topic, version for class member function with bare pointer
             *
             * This version of subscribe is a convenience function for using member functions, and can be used like so:
            \verbatim
            void Foo::callback(const std_msgs::Empty::ConstPtr& message)
            {
            }
            Foo foo_object;
            SubscriberPtr sub = node.subscribe("my_topic", 1, &Foo::callback, &foo_object);
            \endverbatim
             */
            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size,
                                 void (T::*fp)(const boost::shared_ptr<M const>&), T* obj,
                                 const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            /// and the const version
            template <class M, class T>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size,
                                 void (T::*fp)(const boost::shared_ptr<M const>&) const, T* obj,
                                 const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            /**
             * \brief Subscribe to a topic, version for bare function
             *
            \verbatim
            void callback(const std_msgs::Empty::ConstPtr& message){...}
            ros::Subscriber sub = handle.subscribe("my_topic", 1, callback);
            \endverbatim
             */
            template <class M>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size, void (*fp)(const boost::shared_ptr<M const>&),
                                 const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template init<M>(topic, queue_size, fp);
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            /**
             * \brief Subscribe to a topic, version for arbitrary boost::function object
             */
            template <class M>
            SubscriberPtr subscribe(const std::string& topic, uint32_t queue_size, const boost::function<void(const boost::shared_ptr<M const>&)>& callback,
                                 const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(), const ros::TransportHints& transport_hints = ros::TransportHints()) {
                SubscribeOptions ops;
                ops.template init<M>(topic, queue_size, callback);
                ops.tracked_object = tracked_object;
                ops.transport_hints = transport_hints;
                return subscribe(ops);
            }

            /**
             * \brief Subscribe to a topic, version for arbitrary boost::function object
             */
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
