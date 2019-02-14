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

#include "ros_node_lib/topic_manager.h"
#include "ros_node_lib/RosNode.hpp"
#include "ros_node_lib/connection_manager.h"
#include "ros_node_lib/network.h"
#include "ros_node_lib/poll_manager.h"
#include "ros_node_lib/xmlrpc_manager.h"
#include "ros_node_lib/subscription.h"
#include "ros_node_lib/publication.h"
#include "ros_node_lib/subscribe_options.h"
#include "ros_node_lib/advertise_options.h"
#include <XmlRpc.h>
#include <ros/header.h>
#include <ros/transport/transport_tcp.h>
#include <ros/transport/transport_udp.h>

using namespace XmlRpc; // A battle to be fought later

/// \todo Locking can be significantly simplified here once the Node API goes away.

namespace roscan {

void TopicManager::start() {
    std::lock_guard<std::mutex> shutdown_lock{shutting_down_mutex_};
    shutting_down_ = false;

    node_->xmlrpc_manager()->bind("publisherUpdate", boost::bind(&TopicManager::pubUpdateCallback, this, _1, _2));
    node_->xmlrpc_manager()->bind("requestTopic", boost::bind(&TopicManager::requestTopicCallback, this, _1, _2));
    node_->xmlrpc_manager()->bind("getBusStats", boost::bind(&TopicManager::getBusStatsCallback, this, _1, _2));
    node_->xmlrpc_manager()->bind("getBusInfo", boost::bind(&TopicManager::getBusInfoCallback, this, _1, _2));
    node_->xmlrpc_manager()->bind("getSubscriptions", boost::bind(&TopicManager::getSubscriptionsCallback, this, _1, _2));
    node_->xmlrpc_manager()->bind("getPublications", boost::bind(&TopicManager::getPublicationsCallback, this, _1, _2));

    node_->poll_manager()->addPollThreadListener(boost::bind(&TopicManager::processPublishQueues, this));
}

void TopicManager::shutdown() {
    std::lock_guard<std::mutex> shutdown_lock{shutting_down_mutex_};
    if (shutting_down_) {
        return;
    }

    {
        std::lock_guard<std::recursive_mutex> lock1{advertised_topics_mutex_};
        std::lock_guard<std::mutex> lock2{subs_mutex_};
        shutting_down_ = true;
    }

    // actually one should call node_->poll_manager()->removePollThreadListener(), but the connection is not stored above
    node_->poll_manager()->shutdown();

    node_->xmlrpc_manager()->unbind("publisherUpdate");
    node_->xmlrpc_manager()->unbind("requestTopic");
    node_->xmlrpc_manager()->unbind("getBusStats");
    node_->xmlrpc_manager()->unbind("getBusInfo");
    node_->xmlrpc_manager()->unbind("getSubscriptions");
    node_->xmlrpc_manager()->unbind("getPublications");
    {
        std::lock_guard<std::recursive_mutex> adv_lock{advertised_topics_mutex_};

        for (const auto& t: advertised_topics_) {
            if (!t->isDropped()) {
                unregisterPublisher(t->getName());
            }
            t->drop();
        }
        advertised_topics_.clear();
    }

    // unregister all of our subscriptions
    {
        std::lock_guard<std::mutex> subs_lock{subs_mutex_};

        for (const auto& s: subscriptions_) {
            // Remove us as a subscriber from the master
            unregisterSubscriber(s->getName());
            // now, drop our side of the connection
            s->shutdown();
        }
        subscriptions_.clear();
    }
}

void TopicManager::processPublishQueues() {
    std::lock_guard<std::recursive_mutex> lock{advertised_topics_mutex_};

    for (const auto& pub: advertised_topics_) {
        pub->processPublishQueue();
    }
}

void TopicManager::getAdvertisedTopics(std::vector<std::string>& topics) {
    std::lock_guard<std::mutex> lock{advertised_topic_names_mutex_};

    topics.resize(advertised_topic_names_.size());
    std::copy(advertised_topic_names_.cbegin(), advertised_topic_names_.cend(), topics.begin());
}

void TopicManager::getSubscribedTopics(std::vector<std::string>& topics) {
    std::lock_guard<std::mutex> lock{subs_mutex_};

    topics.reserve(subscriptions_.size());
    for (const auto& sub: subscriptions_) {
        topics.push_back(sub->getName());
    }
}

PublicationPtr TopicManager::lookupPublication(const std::string& topic) {
    std::lock_guard<std::recursive_mutex> lock{advertised_topics_mutex_};
    return lookupPublicationWithoutLock(topic);
}

bool md5sumsMatch(const std::string& lhs, const std::string& rhs) {
    return lhs == "*" || rhs == "*" || lhs == rhs;
}

bool TopicManager::addSubCallback(const SubscribeOptions& ops) {
    // spin through the subscriptions and see if we find a match. if so, use it.
    auto found = false;
    auto found_topic = false;

    if (isShuttingDown()) {
        return false;
    }

    SubscriptionPtr sub;
    const auto it = std::find_if(subscriptions_.cbegin(), subscriptions_.cend(), [&ops](const auto& s){ return !s->isDropped() && s->getName() == ops.topic; });
    if (it != subscriptions_.cend()) {
        sub = *it;
        found_topic = true;
        found = md5sumsMatch(ops.md5sum, sub->md5sum());
    }

    if (found_topic && !found) {
        std::stringstream ss;
        ss << "Tried to subscribe to a topic with the same name but different md5sum as a topic that was already subscribed [" << ops.datatype << "/" << ops.md5sum << " vs. " << sub->datatype() << "/" << sub->md5sum() << "]";
        throw ros::ConflictingSubscriptionException(ss.str());
    } else if (found) {
        if (!sub->addCallback(ops.helper, ops.md5sum, ops.callback_queue, ops.queue_size, ops.tracked_object, ops.allow_concurrent_callbacks)) {
            return false;
        }
    }
    return found;
}

// this function has the subscription code that doesn't need to be templated.
bool TopicManager::subscribe(const SubscribeOptions& ops) {
    std::lock_guard<std::mutex> lock{subs_mutex_};

    if (addSubCallback(ops)) {
        return true;
    }

    if (isShuttingDown()) {
        return false;
    }

    if (ops.md5sum.empty()) {
        throw ros::InvalidParameterException("Subscribing to topic [" + ops.topic + "] with an empty md5sum");
    }

    if (ops.datatype.empty()) {
        throw ros::InvalidParameterException("Subscribing to topic [" + ops.topic + "] with an empty datatype");
    }

    if (!ops.helper) {
        throw ros::InvalidParameterException("Subscribing to topic [" + ops.topic + "] without a callback");
    }

    const auto& md5sum = ops.md5sum;
    const auto& datatype = ops.datatype;

    SubscriptionPtr s{boost::make_shared<Subscription>(node_, ops.topic, md5sum, datatype, ops.transport_hints)};
    s->addCallback(ops.helper, ops.md5sum, ops.callback_queue, ops.queue_size, ops.tracked_object, ops.allow_concurrent_callbacks);

    if (!registerSubscriber(s, ops.datatype)) {
        ROS_WARN("couldn't register subscriber on topic [%s]", ops.topic.c_str());
        s->shutdown();
        return false;
    }
    subscriptions_.push_back(s);
    return true;
}

bool TopicManager::advertise(const AdvertiseOptions& ops, const SubscriberCallbacksPtr& callbacks) {
    if (ops.datatype == "*") {
        std::stringstream ss;
        ss << "Advertising with * as the datatype is not allowed.  Topic [" << ops.topic << "]";
        throw ros::InvalidParameterException(ss.str());
    }

    if (ops.md5sum == "*") {
        std::stringstream ss;
        ss << "Advertising with * as the md5sum is not allowed.  Topic [" << ops.topic << "]";
        throw ros::InvalidParameterException(ss.str());
    }

    if (ops.md5sum.empty()) {
        throw ros::InvalidParameterException("Advertising on topic [" + ops.topic + "] with an empty md5sum");
    }

    if (ops.datatype.empty()) {
        throw ros::InvalidParameterException("Advertising on topic [" + ops.topic + "] with an empty datatype");
    }

    if (ops.message_definition.empty()) {
        ROS_WARN("Advertising on topic [%s] with an empty message definition.  Some tools (e.g. rosbag) may not work correctly.", ops.topic.c_str());
    }

    PublicationPtr pub;

    {
        std::lock_guard<std::recursive_mutex> lock{advertised_topics_mutex_};

        if (isShuttingDown()) {
            return false;
        }

        pub = lookupPublicationWithoutLock(ops.topic);
        if (pub && pub->getNumCallbacks() == 0) {
            pub.reset();
        }

        if (pub) {
            if (pub->getMD5Sum() != ops.md5sum) {
                ROS_ERROR("Tried to advertise on topic [%s] with md5sum [%s] and datatype [%s], but the topic is already advertised as md5sum [%s] and datatype [%s]",
                          ops.topic.c_str(), ops.md5sum.c_str(), ops.datatype.c_str(), pub->getMD5Sum().c_str(), pub->getDataType().c_str());
                return false;
            }
            pub->addCallbacks(callbacks);
            return true;
        }

        pub = PublicationPtr{boost::make_shared<Publication>(ops.topic, ops.datatype, ops.md5sum, ops.message_definition, ops.queue_size, ops.latch, ops.has_header)};
        pub->addCallbacks(callbacks);
        advertised_topics_.push_back(pub);
    }

    {
        std::lock_guard<std::mutex> lock{advertised_topic_names_mutex_};
        advertised_topic_names_.push_back(ops.topic);
    }

    // Check whether we've already subscribed to this topic.  If so, we'll do
    // the self-subscription here, to avoid the deadlock that would happen if
    // the ROS thread later got the publisherUpdate with its own XMLRPC URI.
    // The assumption is that advertise() is called from somewhere other
    // than the ROS thread.
    auto found = false;
    SubscriptionPtr sub;
    {
        std::lock_guard<std::mutex> lock{subs_mutex_};

        const auto it = std::find_if(subscriptions_.cbegin(), subscriptions_.cend(), [&ops](const auto& s){ return s->getName() == ops.topic && md5sumsMatch(s->md5sum(), ops.md5sum) && !s->isDropped(); });
        if (it != subscriptions_.cend()) {
            sub = *it;
            found = true;
        }
    }

    if (found) {
        sub->addLocalConnection(pub);
    }

    XmlRpcValue args, result, payload;
    args[0] = node_->getName();
    args[1] = ops.topic;
    args[2] = ops.datatype;
    args[3] = node_->xmlrpc_manager()->getServerURI();
    node_->xmlrpc_manager()->callMaster("registerPublisher", args, result, payload, true);
    return true;
}

bool TopicManager::unadvertise(const std::string& topic, const SubscriberCallbacksPtr& callbacks) {
    PublicationPtr pub;
    V_Publication::const_iterator it;
    {
        std::lock_guard<std::recursive_mutex> lock{advertised_topics_mutex_};

        if (isShuttingDown()) {
            return false;
        }

        it = std::find_if(advertised_topics_.cbegin(), advertised_topics_.cend(), [&topic](const auto& t){ return t->getName() == topic && !t->isDropped(); });
        if (it != advertised_topics_.cend()) {
            pub = *it;
        }
    }

    if (!pub) {
        return false;
    }

    pub->removeCallbacks(callbacks);

    {
        std::lock_guard<std::recursive_mutex> lock{advertised_topics_mutex_};
        if (pub->getNumCallbacks() == 0) {
            unregisterPublisher(pub->getName());
            pub->drop();

            advertised_topics_.erase(it);

            {
                std::lock_guard<std::mutex> lock{advertised_topic_names_mutex_};
                advertised_topic_names_.remove(pub->getName());
            }
        }
    }
    return true;
}

bool TopicManager::unregisterPublisher(const std::string& topic) {
    XmlRpcValue args, result, payload;
    args[0] = node_->getName();
    args[1] = topic;
    args[2] = node_->xmlrpc_manager()->getServerURI();
    node_->xmlrpc_manager()->callMaster("unregisterPublisher", args, result, payload, false);
    return true;
}

bool TopicManager::isTopicAdvertised(const std::string& topic) {
    for (const auto& t: advertised_topics_) {
        if (t->getName() == topic && !t->isDropped()) {
            return true;
        }
    }
    return false;
}

bool TopicManager::registerSubscriber(const SubscriptionPtr& s, const std::string& datatype) {
    XmlRpcValue args, result, payload;
    args[0] = node_->getName();
    args[1] = s->getName();
    args[2] = datatype;
    args[3] = node_->xmlrpc_manager()->getServerURI();

    if (!node_->xmlrpc_manager()->callMaster("registerSubscriber", args, result, payload, true)) {
        return false;
    }

    std::vector<std::string> pub_uris;
    for (int i = 0; i < payload.size(); i++) {
        if (payload[i] != node_->xmlrpc_manager()->getServerURI()) {
            pub_uris.push_back(std::string(payload[i]));
        }
    }

    auto self_subscribed = false;
    PublicationPtr pub;
    const auto& sub_md5sum = s->md5sum();
    // Figure out if we have a local publisher
    {
        std::lock_guard<std::recursive_mutex> lock{advertised_topics_mutex_};
        const auto it = std::find_if(advertised_topics_.cbegin(), advertised_topics_.cend(), [&s](const auto& t){ return t->getName() == s->getName() && !t->isDropped(); });
        if (it != advertised_topics_.cend()) {
            pub = *it;
            if (!md5sumsMatch(pub->getMD5Sum(), sub_md5sum)) {
                ROS_ERROR("md5sum mismatch making local subscription to topic %s.", s->getName().c_str());
                ROS_ERROR("Subscriber expects type %s, md5sum %s", s->datatype().c_str(), s->md5sum().c_str());
                ROS_ERROR("Publisher provides type %s, md5sum %s", pub->getDataType().c_str(), pub->getMD5Sum().c_str());
                return false;
            }
            self_subscribed = true;
        }
    }

    s->pubUpdate(pub_uris);
    if (self_subscribed) {
        s->addLocalConnection(pub);
    }
    return true;
}

bool TopicManager::unregisterSubscriber(const std::string& topic) {
    XmlRpcValue args, result, payload;
    args[0] = node_->getName();
    args[1] = topic;
    args[2] = node_->xmlrpc_manager()->getServerURI();

    node_->xmlrpc_manager()->callMaster("unregisterSubscriber", args, result, payload, false);
    return true;
}

bool TopicManager::pubUpdate(const std::string& topic, const std::vector<std::string>& pubs) {
    SubscriptionPtr sub;
    {
        std::lock_guard<std::mutex> lock{subs_mutex_};

        if (isShuttingDown()) {
            return false;
        }

        // find the subscription
        for (const auto& s: subscriptions_) {
            if (s->getName() == topic && !s->isDropped()) {
                sub = s;
                break;
            }
        }
    }

    if (sub) {
        return sub->pubUpdate(pubs);
    }
    return false;
}

bool TopicManager::requestTopic(const std::string& topic, XmlRpcValue& protos, XmlRpcValue& ret) {
    for (int proto_idx = 0; proto_idx < protos.size(); ++proto_idx) {
        XmlRpcValue proto = protos[proto_idx]; // save typing
        if (proto.getType() != XmlRpcValue::TypeArray) {
            return false;
        }

        if (proto[0].getType() != XmlRpcValue::TypeString) {
            return false;
        }

        std::string proto_name = proto[0];
        if (proto_name == std::string{"TCPROS"}) {
            XmlRpcValue tcpros_params;
            tcpros_params[0] = std::string{"TCPROS"};
            tcpros_params[1] = network::getHost();
            tcpros_params[2] = (int)node_->connection_manager()->getTCPPort();
            ret[0] = 1;
            ret[1] = std::string{};
            ret[2] = tcpros_params;
            return true;
        } else if (proto_name == std::string{"UDPROS"}) {
            if (proto.size() != 5 ||
                proto[1].getType() != XmlRpcValue::TypeBase64 ||
                proto[2].getType() != XmlRpcValue::TypeString ||
                proto[3].getType() != XmlRpcValue::TypeInt ||
                proto[4].getType() != XmlRpcValue::TypeInt) {
                return false;
            }
            std::vector<char> header_bytes = proto[1];
            boost::shared_array<uint8_t> buffer{new uint8_t[header_bytes.size()]};
            memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
            ros::Header h;
            std::string err;
            if (!h.parse(buffer, header_bytes.size(), err)) {
                return false;
            }

            const auto pub_ptr = lookupPublication(topic);
            if (!pub_ptr) {
                return false;
            }

            std::string host = proto[2];
            int port = proto[3];

            std::map<std::string, std::string> m;
            std::string error_msg;
            if (!pub_ptr->validateHeader(h, error_msg)) {
                return false;
            }

            int max_datagram_size = proto[4];
            const auto conn_id = (int)node_->connection_manager()->getNewConnectionID();
            const auto transport = node_->connection_manager()->getUDPServerTransport()->createOutgoing(host, port, conn_id, max_datagram_size);
            if (!transport) {
                return false;
            }
            node_->connection_manager()->udprosIncomingConnection(transport, h);

            XmlRpcValue udpros_params;
            udpros_params[0] = std::string{"UDPROS"};
            udpros_params[1] = network::getHost();
            udpros_params[2] = node_->connection_manager()->getUDPServerTransport()->getServerPort();
            udpros_params[3] = conn_id;
            udpros_params[4] = max_datagram_size;
            m["topic"] = topic;
            m["md5sum"] = pub_ptr->getMD5Sum();
            m["type"] = pub_ptr->getDataType();
            m["callerid"] = node_->getName();
            m["message_definition"] = pub_ptr->getMessageDefinition();
            boost::shared_array<uint8_t> msg_def_buffer;
            uint32_t len;
            ros::Header::write(m, msg_def_buffer, len);
            XmlRpcValue v{msg_def_buffer.get(), (int)len};
            udpros_params[5] = v;
            ret[0] = 1;
            ret[1] = std::string{};
            ret[2] = udpros_params;
            return true;
        }
    }
    return false;
}

void TopicManager::publish(const std::string& topic, const boost::function<ros::SerializedMessage(void)>& serfunc, ros::SerializedMessage& m) {
    std::lock_guard<std::recursive_mutex> lock{advertised_topics_mutex_};

    if (isShuttingDown()) {
        return;
    }

    const auto p = lookupPublicationWithoutLock(topic);
    if (p->hasSubscribers() || p->isLatching()) {
        // Determine what kinds of subscribers we're publishing to.  If they're intraprocess with the same C++ type we can
        // do a no-copy publish.
        auto nocopy = false;
        auto serialize = false;

        // We can only do a no-copy publish if a shared_ptr to the message is provided, and we have type information for it
        if (m.type_info && m.message) {
            p->getPublishTypes(serialize, nocopy, *m.type_info);
        } else {
            serialize = true;
        }

        if (!nocopy) {
            m.message.reset();
            m.type_info = 0;
        }

        if (serialize || p->isLatching()) {
            const auto m2 = serfunc();
            m.buf = m2.buf;
            m.num_bytes = m2.num_bytes;
            m.message_start = m2.message_start;
        }

        p->publish(m);

        // If we're not doing a serialized publish we don't need to signal the pollset.  The write()
        // call inside signal() is actually relatively expensive when doing a nocopy publish.
        if (serialize) {
            node_->poll_manager()->getPollSet().signal();
        }
    } else {
        p->incrementSequence();
    }
}

void TopicManager::incrementSequence(const std::string& topic) {
    const auto pub = lookupPublication(topic);
    if (pub) {
        pub->incrementSequence();
    }
}

bool TopicManager::isLatched(const std::string& topic) {
    const auto pub = lookupPublication(topic);
    if (pub) {
        return pub->isLatched();
    }
    return false;
}

PublicationPtr TopicManager::lookupPublicationWithoutLock(const std::string& topic) {
    PublicationPtr p;
    const auto it = std::find_if(advertised_topics_.cbegin(), advertised_topics_.cend(), [&topic](const auto& t){ return t->getName() == topic && !t->isDropped(); });
    if (it != advertised_topics_.cend()) {
        p = *it;
    }
    return p;
}

bool TopicManager::unsubscribe(const std::string& topic, const ros::SubscriptionCallbackHelperPtr& helper) {
    SubscriptionPtr sub;
    {
        std::lock_guard<std::mutex> lock{subs_mutex_};

        if (isShuttingDown()) {
            return false;
        }

        const auto it = std::find_if(subscriptions_.cbegin(), subscriptions_.cend(), [&topic](const auto& s){ return s->getName() == topic; });
        if (it != subscriptions_.cend()) {
            sub = *it;
        }
    }

    if (!sub) {
        return false;
    }

    sub->removeCallback(helper);

    if (sub->getNumCallbacks() == 0) {
        // nobody is left. blow away the subscription.
        {
            std::lock_guard<std::mutex> lock{subs_mutex_};

            const auto it = std::find_if(subscriptions_.cbegin(), subscriptions_.cend(), [&topic](const auto& s){ return s->getName() == topic; });
            if (it != subscriptions_.cend()) {
                subscriptions_.erase(it);
            }
            unregisterSubscriber(topic);
        }
        sub->shutdown();
        return true;
    }
    return true;
}

size_t TopicManager::getNumSubscribers(const std::string& topic) {
    std::lock_guard<std::recursive_mutex> lock{advertised_topics_mutex_};

    if (isShuttingDown()) {
        return 0;
    }

    const auto p = lookupPublicationWithoutLock(topic);
    if (p) {
        return p->getNumSubscribers();
    }
    return 0;
}

size_t TopicManager::getNumSubscriptions() {
    std::lock_guard<std::mutex> lock{subs_mutex_};
    return subscriptions_.size();
}

size_t TopicManager::getNumPublishers(const std::string& topic) {
    std::lock_guard<std::mutex> lock{subs_mutex_};

    if (isShuttingDown()) {
        return 0;
    }

    for (const auto& s: subscriptions_) {
        if (!s->isDropped() && s->getName() == topic) {
            return s->getNumPublishers();
        }
    }
    return 0;
}

void TopicManager::getBusStats(XmlRpcValue& stats) {
    XmlRpcValue publish_stats, subscribe_stats, service_stats;
    // force these guys to be arrays, even if we don't populate them
    publish_stats.setSize(0);
    subscribe_stats.setSize(0);
    service_stats.setSize(0);

    uint32_t pidx = 0;
    {
        std::lock_guard<std::recursive_mutex> lock{advertised_topics_mutex_};
        for (const auto& t: advertised_topics_) {
            publish_stats[pidx++] = t->getStats();
        }
    }

    {
        uint32_t sidx = 0;

        std::lock_guard<std::mutex> lock{subs_mutex_};
        for (const auto& s: subscriptions_) {
            subscribe_stats[sidx++] = s->getStats();
        }
    }
    stats[0] = publish_stats;
    stats[1] = subscribe_stats;
    stats[2] = service_stats;
}

void TopicManager::getBusInfo(XmlRpcValue& info) {
    // force these guys to be arrays, even if we don't populate them
    info.setSize(0);

    {
        std::lock_guard<std::recursive_mutex> lock{advertised_topics_mutex_};

        for (const auto& t: advertised_topics_) {
            t->getInfo(info);
        }
    }

    {
        std::lock_guard<std::mutex> lock{subs_mutex_};

        for (const auto& s: subscriptions_) {
            s->getInfo(info);
        }
    }
}

void TopicManager::getSubscriptions(XmlRpcValue& subs) {
    // force these guys to be arrays, even if we don't populate them
    subs.setSize(0);

    {
        std::lock_guard<std::mutex> lock{subs_mutex_};

        uint32_t sidx = 0;
        for (const auto& s: subscriptions_) {
            XmlRpcValue sub;
            sub[0] = s->getName();
            sub[1] = s->datatype();
            subs[sidx++] = sub;
        }
    }
}

void TopicManager::getPublications(XmlRpcValue& pubs) {
    // force these guys to be arrays, even if we don't populate them
    pubs.setSize(0);

    {
        std::lock_guard<std::recursive_mutex> lock{advertised_topics_mutex_};

        uint32_t sidx = 0;
        for (const auto& t: advertised_topics_) {
            XmlRpcValue pub;
            pub[0] = t->getName();
            pub[1] = t->getDataType();
            pubs[sidx++] = pub;
        }
    }
}

void TopicManager::pubUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    std::vector<std::string> pubs;
    for (int idx = 0; idx < params[2].size(); ++idx) {
        pubs.push_back(params[2][idx]);
    }
    if (pubUpdate(params[1], pubs)) {
        result = xmlrpc::responseInt(1, "", 0);
    } else {
        result = xmlrpc::responseInt(0, "SOME KIND OF ERROR OH NO", 0);
    }
}

void TopicManager::requestTopicCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    if (!requestTopic(params[1], params[2], result)) {
        result = xmlrpc::responseInt(0, "SOME KINDA ERROR OH NO", 0);
    }
}

void TopicManager::getBusStatsCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    (void)params;
    result[0] = 1;
    result[1] = std::string{""};
    XmlRpcValue response;
    getBusStats(result);
    result[2] = response;
}

void TopicManager::getBusInfoCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    (void)params;
    result[0] = 1;
    result[1] = std::string{""};
    XmlRpcValue response;
    getBusInfo(response);
    result[2] = response;
}

void TopicManager::getSubscriptionsCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    (void)params;
    result[0] = 1;
    result[1] = std::string{"subscriptions"};
    XmlRpcValue response;
    getSubscriptions(response);
    result[2] = response;
}

void TopicManager::getPublicationsCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    (void)params;
    result[0] = 1;
    result[1] = std::string{"publications"};
    XmlRpcValue response;
    getPublications(response);
    result[2] = response;
}

} // namespace roscan
