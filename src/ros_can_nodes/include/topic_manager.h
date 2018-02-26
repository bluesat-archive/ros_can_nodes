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
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#ifndef ROSCAN_TOPIC_MANAGER_H
#define ROSCAN_TOPIC_MANAGER_H

#include "common.h"
#include "RosCanNode.h"
#include "connection_manager.h"
#include "poll_manager.h"
#include "xmlrpc_manager.h"
#include "subscription.h"
#include "publication.h"
#include "subscribe_options.h"
#include <XmlRpcValue.h>
#include <ros/rosout_appender.h>
#include <ros/serialization.h>
#include <ros/advertise_options.h>

namespace roscan {

typedef std::vector<std::string> V_string;

class TopicManager {
    public:
        TopicManager() : shutting_down_(false) {}
        ~TopicManager() { shutdown(); }

        void start(const RosCanNodePtr& node);
        void shutdown();

        bool subscribe(const SubscribeOptions& ops);
        bool unsubscribe(const std::string& _topic, const ros::SubscriptionCallbackHelperPtr& helper);

        bool advertise(const ros::AdvertiseOptions& ops, const SubscriberCallbacksPtr& callbacks);
        bool unadvertise(const std::string& topic, const SubscriberCallbacksPtr& callbacks);

        // Get the list of topics advertised by this node
        void getAdvertisedTopics(V_string& topics);

        // Get the list of topics subscribed to by this node
        void getSubscribedTopics(V_string& topics);

        // Lookup an advertised topic.
        // This method iterates over advertised_topics, looking for one with name
        // matching the given topic name.  The advertised_topics_mutex is locked
        // during this search.  This method is only used internally.
        // Returns pointer to the matching Publication, NULL if none is found.
        PublicationPtr lookupPublication(const std::string& topic);

        // Return the number of subscribers a node has for a particular topic
        size_t getNumSubscribers(const std::string& _topic);
        size_t getNumSubscriptions();

        // Return the number of publishers connected to this node on a particular topic
        size_t getNumPublishers(const std::string& _topic);

        template <typename M>
        void publish(const std::string& topic, const M& message) {
            ros::SerializedMessage m;
            publish(topic, boost::bind(ros::serialization::serializeMessage<M>, boost::ref(message)), m);
        }

        void publish(const std::string& _topic, const boost::function<ros::SerializedMessage(void)>& serfunc, ros::SerializedMessage& m);

        void incrementSequence(const std::string& _topic);
        bool isLatched(const std::string& topic);

    private:
        // if it finds a pre-existing subscription to the same topic and of the
        // same message type, it appends the Functor to the callback vector for
        // that subscription. otherwise, it returns false, indicating that a new
        // subscription needs to be created.
        bool addSubCallback(const SubscribeOptions& ops);

        // Request a topic
        // Negotiate a subscriber connection on a topic.
        // @param topic The topic of interest.
        // @param protos List of transport protocols, in preference order
        // @param ret Return value
        // @return true on success, false otherwise
        // @todo Consider making this private
        bool requestTopic(const std::string& topic, XmlRpc::XmlRpcValue& protos, XmlRpc::XmlRpcValue& ret);

        // Must lock the advertised topics mutex before calling this function
        bool isTopicAdvertised(const std::string& topic);

        bool registerSubscriber(const SubscriptionPtr& s, const std::string& datatype);
        bool unregisterSubscriber(const std::string& topic);
        bool unregisterPublisher(const std::string& topic);

        PublicationPtr lookupPublicationWithoutLock(const std::string& topic);

        void processPublishQueues();

        // Compute the statistics for the node's connectivity
        // This is the implementation of the xml-rpc getBusStats function;
        // it populates the XmlRpcValue object sent to it with various statistics
        // about the node's connectivity, bandwidth utilization, etc.
        void getBusStats(XmlRpc::XmlRpcValue& stats);

        // Compute the info for the node's connectivity
        // This is the implementation of the xml-rpc getBusInfo function;
        // it populates the XmlRpcValue object sent to it with various info
        // about the node's connectivity.
        void getBusInfo(XmlRpc::XmlRpcValue& info);

        // Return the list of subcriptions for the node
        // This is the implementation of the xml-rpc getSubscriptions
        // function; it populates the XmlRpcValue object sent to it with the
        // list of subscribed topics and their datatypes.
        void getSubscriptions(XmlRpc::XmlRpcValue& subscriptions);

        // Return the list of advertised topics for the node
        // This is the implementation of the xml-rpc getPublications
        // function; it populates the XmlRpcValue object sent to it with the
        // list of advertised topics and their datatypes.
        void getPublications(XmlRpc::XmlRpcValue& publications);

        // Update local publisher lists.
        // Use this method to update address information for publishers on a
        // given topic.
        // @param topic The topic of interest
        // @param pubs The list of publishers to update.
        // Return true on success, false otherwise.
        bool pubUpdate(const std::string& topic, const std::vector<std::string>& pubs);

        void pubUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
        void requestTopicCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
        void getBusStatsCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
        void getBusInfoCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
        void getSubscriptionsCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
        void getPublicationsCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

        bool isShuttingDown() { return shutting_down_; }

        boost::mutex subs_mutex_;
        L_Subscription subscriptions_;

        boost::recursive_mutex advertised_topics_mutex_;
        V_Publication advertised_topics_;
        std::list<std::string> advertised_topic_names_;
        boost::mutex advertised_topic_names_mutex_;

        volatile bool shutting_down_;
        boost::mutex shutting_down_mutex_;

        RosCanNodePtr node_;

        PollManagerPtr poll_manager_;
        ConnectionManagerPtr connection_manager_;
        XMLRPCManagerPtr xmlrpc_manager_;
};

} // namespace roscan

#endif // ROSCAN_TOPIC_MANAGER_H
